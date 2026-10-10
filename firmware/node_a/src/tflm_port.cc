/**
 * @file tflm_port.cc
 * @brief TensorFlow Lite Micro platform hooks for Node A: its messages go to our log.
 */
#include <stdarg.h>
#include <stdio.h>

#include "log.h"
#include "tensorflow/lite/micro/debug_log.h"

namespace
{
constexpr size_t kLineBytes = 128U;
} // namespace

extern "C" void DebugLog(const char *format, va_list args)
{
    char line[kLineBytes];

    (void)vsnprintf(line, sizeof(line), format, args);
    LOG_WARN("tflm: %s", line);
}

extern "C" int DebugVsnprintf(char *buffer, size_t buf_size, const char *format, va_list vlist)
{
    return vsnprintf(buffer, buf_size, format, vlist);
}
