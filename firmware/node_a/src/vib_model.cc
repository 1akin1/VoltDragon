/**
 * @file vib_model.cc
 * @brief The INT8 vibration classifier, run with TensorFlow Lite Micro.
 *
 * The interpreter and op resolver are C++ objects. Our startup code runs no
 * static constructors (the linker script rejects them), so they are built
 * with placement new into static storage in vib_model_init().
 */
#include "vib_model.h"

#include <math.h>
#include <string.h>

#include <new>

#include "log.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "vib_model_data.h"

namespace
{

/*
 * On Node A the model uses 1252 bytes (vib_model_arena_used(), logged at start-up).
 * The interpreter's bookkeeping holds pointers, so a 64-bit host build (the unit
 * test) needs more and sets VIB_MODEL_ARENA_BYTES.
 */
#ifndef VIB_MODEL_ARENA_BYTES
#define VIB_MODEL_ARENA_BYTES 2048U
#endif
constexpr size_t kArenaBytes = VIB_MODEL_ARENA_BYTES;
constexpr int kOps = 2;

using Resolver = tflite::MicroMutableOpResolver<kOps>;

alignas(16) uint8_t s_arena[kArenaBytes];
alignas(Resolver) uint8_t s_resolver_storage[sizeof(Resolver)];
alignas(tflite::MicroInterpreter) uint8_t s_interpreter_storage[sizeof(tflite::MicroInterpreter)];

tflite::MicroInterpreter *s_interpreter = nullptr;
TfLiteTensor *s_input = nullptr;
TfLiteTensor *s_output = nullptr;

/** True for an int8 tensor of shape [1, length]. */
bool shape_is(const TfLiteTensor *t, int length)
{
    return (t != nullptr) && (t->type == kTfLiteInt8) && (t->dims != nullptr) &&
           (t->dims->size == 2) && (t->dims->data[0] == 1) && (t->dims->data[1] == length);
}

} // namespace

extern "C" bool vib_model_init(void)
{
    if (s_interpreter != nullptr)
    {
        return true;
    }

    const tflite::Model *model = tflite::GetModel(vib_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        LOG_ERROR("ai: model schema %u, expected %d", (unsigned int)model->version(),
                  TFLITE_SCHEMA_VERSION);
        return false;
    }

    Resolver *resolver = new (s_resolver_storage) Resolver();
    if ((resolver->AddFullyConnected() != kTfLiteOk) || (resolver->AddSoftmax() != kTfLiteOk))
    {
        LOG_ERROR("ai: could not register the kernels");
        return false;
    }

    tflite::MicroInterpreter *interpreter =
        new (s_interpreter_storage) tflite::MicroInterpreter(model, *resolver, s_arena, kArenaBytes);
    if (interpreter->AllocateTensors() != kTfLiteOk)
    {
        LOG_ERROR("ai: tensor arena of %u bytes is too small", (unsigned int)kArenaBytes);
        return false;
    }

    TfLiteTensor *input = interpreter->input(0);
    TfLiteTensor *output = interpreter->output(0);
    if (!shape_is(input, (int)VIB_FEATURES) || !shape_is(output, (int)VIB_CLASS_COUNT) ||
        (input->params.scale <= 0.0f) || (output->params.scale <= 0.0f))
    {
        LOG_ERROR("ai: the model's input or output does not match the firmware");
        return false;
    }

    s_input = input;
    s_output = output;
    s_interpreter = interpreter;
    return true;
}

extern "C" void vib_model_quantise(const float features[VIB_FEATURES], int8_t input[VIB_FEATURES])
{
    const float scale = (s_input != nullptr) ? s_input->params.scale : 1.0f;
    const int32_t zero = (s_input != nullptr) ? s_input->params.zero_point : 0;

    for (uint32_t i = 0U; i < VIB_FEATURES; ++i)
    {
        int32_t q = (int32_t)roundf(features[i] / scale) + zero;
        q = (q > INT8_MAX) ? INT8_MAX : ((q < INT8_MIN) ? INT8_MIN : q);
        input[i] = (int8_t)q;
    }
}

extern "C" bool vib_model_run(const int8_t input[VIB_FEATURES], int8_t output[VIB_CLASS_COUNT])
{
    if (s_interpreter == nullptr)
    {
        return false;
    }
    (void)memcpy(s_input->data.int8, input, VIB_FEATURES);
    if (s_interpreter->Invoke() != kTfLiteOk)
    {
        return false;
    }
    (void)memcpy(output, s_output->data.int8, VIB_CLASS_COUNT);
    return true;
}

extern "C" float vib_model_probability(int8_t score)
{
    if (s_output == nullptr)
    {
        return 0.0f;
    }
    return (float)((int32_t)score - s_output->params.zero_point) * s_output->params.scale;
}

extern "C" uint32_t vib_model_arena_used(void)
{
    return (s_interpreter != nullptr) ? (uint32_t)s_interpreter->arena_used_bytes() : 0U;
}
