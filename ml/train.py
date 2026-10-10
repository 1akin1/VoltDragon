"""Trains the vibration classifier, quantises it to INT8 and reports FP32 vs INT8 accuracy.

Usage (Linux/WSL, with the 'ml' extra installed):
    python -m ml.dataset --out build/ml/dataset.npz
    python -m ml.train --dataset build/ml/dataset.npz

Steps:
1. Cut each flight into the windows Node A classifies (ml/features.py) and
   label each window with the class of at least 3/4 of its samples (windows
   that straddle a fault onset are left out of training and of the window
   metrics, but not of the alarm evaluation).
2. Split by flight (70/15/15 %), so that no flight contributes to both
   training and test data.
3. Train a small dense network (42 -> 32 -> 16 -> 3) on standardised
   features, then fold the standardisation into the first layer so the
   model takes the raw features.
4. Convert to TFLite twice: FP32, and full-integer INT8 (int8 input and
   output, calibrated on training windows). Both are evaluated with the
   TFLite reference kernels, the same arithmetic TFLite Micro uses on Node A.
5. Evaluate on the test flights: window accuracy and confusion matrices, and
   the debounced alarm (detection latency, missed faults, false alarms).
6. Fly SOAK_FLIGHTS further fault-free flights (new seed, not used in training)
   to estimate the false-alarm rate over hours rather than minutes.

Writes:
    ml/models/vibration_{fp32,int8}.tflite, ml/models/metrics.json
    docs/vibration-model-report.md            the accuracy report
    firmware/node_a/src/vib_model_data.{c,h}  the INT8 model as a C array
    tests/unit/vib_vectors.h                  test vectors for the C features and model
"""

from __future__ import annotations

import argparse
import json
import os
import platform
from pathlib import Path

os.environ.setdefault("TF_CPP_MIN_LOG_LEVEL", "2")

import numpy as np  # noqa: E402
import tensorflow as tf  # noqa: E402

from ml import features as F  # noqa: E402
from ml.dataset import collect  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
MODELS = REPO / "ml" / "models"
CLASSES = ("nominal", "imbalance", "bearing")
LABEL_PURITY = 0.75
SEED = 7
SOAK_FLIGHTS = 90
SOAK_FLIGHT_S = 120.0
# Seed 1000 served to diagnose false alarms from the weakest faults (docs/edge-ai.md); the
# reported soak test uses flights that no design decision has seen.
SOAK_SEED = 2000


# ---------------------------------------------------------------------------- data


def load(path: Path) -> dict:
    data = dict(np.load(path))
    data["meta"] = json.loads(str(data["meta"]))
    return data


def flight_windows(data: dict, flight: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """Features, labels (-1 if mixed) and end times of one flight's windows, and its metadata."""
    rows = data["flight"] == flight
    samples = F.to_physical(data["accel_mg"][rows], data["gyro_mdps"][rows])
    labels = data["label"][rows]
    times = data["t"][rows]
    starts = F.windows(samples)
    x = np.stack([F.window_features(samples[s:s + F.WINDOW]) for s in starts])
    y = np.full(len(starts), -1, dtype=np.int64)
    for i, s in enumerate(starts):
        counts = np.bincount(labels[s:s + F.WINDOW], minlength=len(CLASSES))
        if counts.max() >= LABEL_PURITY * F.WINDOW:
            y[i] = int(counts.argmax())
    end_t = times[starts + F.WINDOW - 1]
    return x, y, end_t, data["meta"][flight]


def split(flight: int) -> str:
    bucket = flight % 20
    return "train" if bucket < 14 else ("val" if bucket < 17 else "test")


# ---------------------------------------------------------------------------- model


def build_model(n_features: int) -> tf.keras.Model:
    reg = tf.keras.regularizers.l2(1e-4)
    return tf.keras.Sequential([
        tf.keras.Input(shape=(n_features,), name="features"),
        tf.keras.layers.Dense(32, activation="relu", kernel_regularizer=reg),
        tf.keras.layers.Dense(16, activation="relu", kernel_regularizer=reg),
        tf.keras.layers.Dense(len(CLASSES), activation="softmax"),
    ])


def fold_standardisation(
    model: tf.keras.Model, mean: np.ndarray, std: np.ndarray
) -> tf.keras.Model:
    """The same network taking raw features: (x - mean) / std folded into the first layer."""
    folded = build_model(len(mean))
    folded.set_weights(model.get_weights())
    first = folded.layers[0]
    kernel, bias = first.get_weights()
    first.set_weights([kernel / std[:, None], bias - (mean / std) @ kernel])
    return folded


def convert(model: tf.keras.Model, calibration: np.ndarray | None) -> bytes:
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    if calibration is not None:
        def representative():
            for row in calibration:
                yield [row[None, :].astype(np.float32)]

        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        converter.representative_dataset = representative
        converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        converter.inference_input_type = tf.int8
        converter.inference_output_type = tf.int8
    return converter.convert()


class TfliteModel:
    """Runs a .tflite model with the reference kernels (as TFLite Micro does)."""

    def __init__(self, flatbuffer: bytes) -> None:
        self.interpreter = tf.lite.Interpreter(
            model_content=flatbuffer,
            experimental_op_resolver_type=tf.lite.experimental.OpResolverType.BUILTIN_REF,
        )
        self.interpreter.allocate_tensors()
        self.input = self.interpreter.get_input_details()[0]
        self.output = self.interpreter.get_output_details()[0]
        self.quantised = self.input["dtype"] == np.int8

    def quantise_input(self, x: np.ndarray) -> np.ndarray:
        scale, zero = self.input["quantization"]
        # Rounds half away from zero, like roundf() on Node A.
        q = np.sign(x / scale) * np.floor(np.abs(x / scale) + 0.5) + zero
        return np.clip(q, -128, 127).astype(np.int8)

    def raw(self, x: np.ndarray) -> np.ndarray:
        """Output tensors for each row of @x (int8 for the quantised model)."""
        out = []
        for row in x:
            value = self.quantise_input(row) if self.quantised else row.astype(np.float32)
            self.interpreter.set_tensor(self.input["index"], value[None, :])
            self.interpreter.invoke()
            out.append(self.interpreter.get_tensor(self.output["index"])[0].copy())
        return np.array(out)

    def probabilities(self, x: np.ndarray) -> np.ndarray:
        raw = self.raw(x)
        if not self.quantised:
            return raw
        scale, zero = self.output["quantization"]
        return (raw.astype(np.float32) - zero) * scale


# ---------------------------------------------------------------------------- evaluation


def confusion(y: np.ndarray, predicted: np.ndarray) -> np.ndarray:
    matrix = np.zeros((len(CLASSES), len(CLASSES)), dtype=np.int64)
    for truth, guess in zip(y, predicted, strict=True):
        matrix[truth, guess] += 1
    return matrix


def window_metrics(y: np.ndarray, probabilities: np.ndarray) -> dict:
    predicted = probabilities.argmax(axis=1)
    matrix = confusion(y, predicted)
    recall = matrix.diagonal() / np.maximum(matrix.sum(axis=1), 1)
    precision = matrix.diagonal() / np.maximum(matrix.sum(axis=0), 1)
    return {
        "accuracy": float((predicted == y).mean()),
        "recall": dict(zip(CLASSES, recall.tolist(), strict=True)),
        "precision": dict(zip(CLASSES, precision.tolist(), strict=True)),
        "confusion": matrix.tolist(),
    }


def alarm_metrics(flights: list[tuple], model: TfliteModel) -> dict:
    """Runs the debounced alarm over whole test flights."""
    latencies, missed, wrong_class, false_alarms = [], 0, 0, 0
    nominal_s = 0.0
    for x, _, end_t, meta in flights:
        alarm_filter = F.AlarmFilter()
        alarms = [alarm_filter.update(int(c)) for c in model.probabilities(x).argmax(axis=1)]
        fault = meta["fault"]
        onset = fault["onset_s"] if fault else float("inf")
        before = [a for a, t in zip(alarms, end_t, strict=True) if t < onset]
        nominal_s += min(onset, float(end_t[-1])) - float(end_t[0])
        # A false alarm: one rising edge of the alarm before any fault exists.
        false_alarms += sum(1 for a, b in zip([0, *before], before, strict=False) if a == 0 and b)
        if fault is None:
            continue
        kind = CLASSES.index(fault["kind"])
        after = [(a, t) for a, t in zip(alarms, end_t, strict=True) if t >= onset]
        hit = next((t for a, t in after if a == kind), None)
        if hit is None:
            missed += 1
            if any(a not in (0, kind) for a, _ in after):
                wrong_class += 1
        else:
            latencies.append(float(hit - onset))
    lat = np.array(latencies) if latencies else np.array([np.nan])
    return {
        "fault_flights": len(latencies) + missed,
        "detected": len(latencies),
        "missed": missed,
        "missed_with_wrong_class": wrong_class,
        "latency_s": {"median": float(np.median(lat)), "p95": float(np.percentile(lat, 95)),
                      "max": float(lat.max())},
        "false_alarms": false_alarms,
        "fault_free_hours": nominal_s / 3600.0,
    }


# ---------------------------------------------------------------------------- outputs


def c_array(data: bytes, per_line: int = 12) -> str:
    lines = []
    for i in range(0, len(data), per_line):
        lines.append("    " + ", ".join(f"0x{b:02X}" for b in data[i:i + per_line]) + ",")
    return "\n".join(lines)


def write_model_source(flatbuffer: bytes) -> None:
    src = REPO / "firmware" / "node_a" / "src"
    (src / "vib_model_data.h").write_text(
        "/**\n"
        " * @file vib_model_data.h\n"
        " * @brief The INT8 vibration classifier as a TFLite flatbuffer.\n"
        " *\n"
        " * Generated by ml/train.py; do not edit.\n"
        " */\n"
        "#ifndef VIB_MODEL_DATA_H\n#define VIB_MODEL_DATA_H\n\n#include <stdint.h>\n\n"
        "#ifdef __cplusplus\nextern \"C\" {\n#endif\n\n"
        "extern const uint8_t vib_model_data[];\n"
        "extern const uint32_t vib_model_data_len;\n\n"
        "#ifdef __cplusplus\n}\n#endif\n\n#endif /* VIB_MODEL_DATA_H */\n",
        encoding="utf-8", newline="\n",
    )
    (src / "vib_model_data.c").write_text(
        "/**\n"
        " * @file vib_model_data.c\n"
        " * @brief The INT8 vibration classifier as a TFLite flatbuffer.\n"
        " *\n"
        " * Generated by ml/train.py; do not edit.\n"
        " */\n"
        '#include "vib_model_data.h"\n\n'
        "/* TFLite Micro reads the flatbuffer in place: it must be 16-byte aligned. */\n"
        "const uint8_t vib_model_data[] __attribute__((aligned(16))) = {\n"
        f"{c_array(flatbuffer)}\n}};\n\n"
        f"const uint32_t vib_model_data_len = {len(flatbuffer)}U;\n",
        encoding="utf-8", newline="\n",
    )


def write_vectors(data: dict, test_flights: list[int], model: TfliteModel) -> None:
    """Two test windows per class: raw samples, features, quantised input and output."""
    chosen = []
    for cls in range(len(CLASSES)):
        found = 0
        for flight in test_flights:
            rows = np.flatnonzero(data["flight"] == flight)
            labels = data["label"][rows]
            for start in F.windows(rows)[::7]:
                if np.all(labels[start:start + F.WINDOW] == cls):
                    chosen.append((rows[start:start + F.WINDOW], cls))
                    found += 1
                    break
            if found == 2:
                break

    def ints(values: np.ndarray) -> str:
        return ", ".join(str(int(v)) for v in values)

    def floats(values: np.ndarray) -> str:
        return ", ".join(f"{float(v):.7g}f" for v in values)

    blocks = []
    for rows, cls in chosen:
        accel, gyro = data["accel_mg"][rows], data["gyro_mdps"][rows]
        feats = F.window_features(F.to_physical(accel, gyro))
        q_in = model.quantise_input(feats)
        q_out = model.raw(feats[None, :])[0]
        blocks.append(
            "    {\n"
            f"        .label = {cls},\n"
            f"        .accel_mg = {{ {ints(accel.reshape(-1))} }},\n"
            f"        .gyro_mdps = {{ {ints(gyro.reshape(-1))} }},\n"
            f"        .features = {{ {floats(feats)} }},\n"
            f"        .input = {{ {ints(q_in)} }},\n"
            f"        .output = {{ {ints(q_out)} }},\n"
            "    },"
        )
    (REPO / "tests" / "unit" / "vib_vectors.h").write_text(
        "/**\n"
        " * @file vib_vectors.h\n"
        " * @brief Test vectors for the vibration features and model, from the Python reference.\n"
        " *\n"
        " * Generated by ml/train.py from test-set windows; do not edit. Samples are\n"
        " * interleaved x, y, z per sample, in the units Node A's IMU driver reports.\n"
        " */\n"
        "#ifndef VIB_VECTORS_H\n#define VIB_VECTORS_H\n\n#include <stdint.h>\n\n"
        f"#define VIB_VECTOR_COUNT ({len(blocks)}U)\n\n"
        "typedef struct\n{\n"
        "    int32_t label;\n"
        f"    int32_t accel_mg[{F.WINDOW * 3}];\n"
        f"    int32_t gyro_mdps[{F.WINDOW * 3}];\n"
        f"    float   features[{F.FEATURES}];\n"
        f"    int8_t  input[{F.FEATURES}];\n"
        f"    int8_t  output[{len(CLASSES)}];\n"
        "} vib_vector_t;\n\n"
        "static const vib_vector_t VIB_VECTORS[VIB_VECTOR_COUNT] = {\n"
        + "\n".join(blocks)
        + "\n};\n\n#endif /* VIB_VECTORS_H */\n",
        encoding="utf-8", newline="\n",
    )


def pct(value: float) -> str:
    return f"{100.0 * value:.2f} %"


def confusion_table(matrix: list[list[int]]) -> str:
    head = "| true \\ predicted | " + " | ".join(CLASSES) + " |\n"
    head += "|---|" + "---|" * len(CLASSES) + "\n"
    rows = "".join(
        f"| {CLASSES[i]} | " + " | ".join(str(v) for v in row) + " |\n"
        for i, row in enumerate(matrix)
    )
    return head + rows


def write_report(metrics: dict) -> None:
    fp32, int8 = metrics["fp32"], metrics["int8"]
    lines = [
        "# Vibration classifier: FP32 vs INT8 report",
        "",
        "Generated by `ml/train.py`; do not edit. Background and design: "
        "[edge-ai.md](edge-ai.md).",
        "",
        "## Data",
        "",
        f"- {metrics['data']['flights']} simulated flights of {metrics['data']['flight_s']:.0f} s "
        "(plant model only), split by flight into training, validation and test sets.",
        f"- Windows of {F.WINDOW} samples (0.64 s) every {F.HOP} samples (0.32 s); "
        f"{F.FEATURES} spectral features per window.",
        "",
        "| Set | Flights | Windows | nominal | imbalance | bearing |",
        "|---|---|---|---|---|---|",
    ]
    for name, info in metrics["data"]["sets"].items():
        counts = " | ".join(str(c) for c in info["per_class"])
        lines.append(f"| {name} | {info['flights']} | {info['windows']} | {counts} |")
    lines += [
        "",
        "## Model",
        "",
        "Dense network 42 → 32 (ReLU) → 16 (ReLU) → 3 (softmax), with the feature "
        "standardisation folded into the first layer.",
        "",
        "| | FP32 | INT8 |",
        "|---|---|---|",
        f"| Parameters | {metrics['parameters']} | {metrics['parameters']} |",
        f"| Flatbuffer size | {metrics['size']['fp32']} bytes | {metrics['size']['int8']} bytes |",
        f"| Test window accuracy | {pct(fp32['accuracy'])} | {pct(int8['accuracy'])} |",
    ]
    for cls in CLASSES:
        lines.append(f"| Recall: {cls} | {pct(fp32['recall'][cls])} | {pct(int8['recall'][cls])} |")
    for cls in CLASSES:
        lines.append(
            f"| Precision: {cls} | {pct(fp32['precision'][cls])} | {pct(int8['precision'][cls])} |"
        )
    agreement = metrics["agreement"]
    lines += [
        "",
        f"**Accuracy difference (INT8 − FP32): "
        f"{100.0 * (int8['accuracy'] - fp32['accuracy']):+.2f} percentage points.** "
        f"The two models predict the same class for {pct(agreement['same_class'])} of the "
        f"test windows; the largest difference in a class probability is "
        f"{agreement['max_probability_diff']:.3f} (mean {agreement['mean_probability_diff']:.4f}).",
        "",
        "### Confusion matrix, FP32 (test windows)",
        "",
        confusion_table(fp32["confusion"]),
        "### Confusion matrix, INT8 (test windows)",
        "",
        confusion_table(int8["confusion"]),
        "## Alarm on the test flights (INT8, as on Node A)",
        "",
        f"The alarm is raised when {F.FAULT_WINDOWS} of the last {F.HISTORY} windows show a "
        f"fault and cleared after {F.CLEAR_WINDOWS} nominal windows in a row. Latency is "
        "measured from the fault onset to the end of the window that raises the alarm "
        "with the correct class; transport to the ground station adds up to about 0.1 s.",
        "",
        "| | |",
        "|---|---|",
    ]
    alarm = metrics["alarm_int8"]
    lines += [
        f"| Fault flights detected | {alarm['detected']} of {alarm['fault_flights']} |",
        f"| Missed (of which raised with the wrong class) | {alarm['missed']} "
        f"({alarm['missed_with_wrong_class']}) |",
        f"| Detection latency: median / 95th percentile / max | "
        f"{alarm['latency_s']['median']:.2f} s / {alarm['latency_s']['p95']:.2f} s / "
        f"{alarm['latency_s']['max']:.2f} s |",
        f"| False alarms | {alarm['false_alarms']} in {alarm['fault_free_hours']:.2f} h "
        "of fault-free flight |",
        "",
        "### Fault-free soak test",
        "",
        f"{SOAK_FLIGHTS} more fault-free flights of {SOAK_FLIGHT_S:.0f} s, with conditions drawn "
        f"like the training flights' but from another seed ({SOAK_SEED}):",
        "",
        "| | |",
        "|---|---|",
        f"| Windows classified as a fault | {metrics['soak_window_errors_int8']} of "
        f"{metrics['soak_windows']} |",
        f"| False alarms | {metrics['soak_int8']['false_alarms']} in "
        f"{metrics['soak_int8']['fault_free_hours']:.2f} h |",
        "",
        "## Reproducing",
        "",
        "```sh",
        f"python -m ml.dataset --flights {metrics['data']['flights']} "
        f"--duration {metrics['data']['flight_s']:.0f} --seed {metrics['data']['seed']} "
        "--out build/ml/dataset.npz",
        "python -m ml.train --dataset build/ml/dataset.npz",
        "```",
        "",
        f"TensorFlow {metrics['tensorflow']}, Python {metrics['python']}, training seed {SEED}.",
        "",
    ]
    (REPO / "docs" / "vibration-model-report.md").write_text(
        "\n".join(lines), encoding="utf-8", newline="\n"
    )


# ---------------------------------------------------------------------------- main


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dataset", type=Path, default=REPO / "build" / "ml" / "dataset.npz")
    parser.add_argument("--epochs", type=int, default=150)
    args = parser.parse_args(argv)

    tf.keras.utils.set_random_seed(SEED)
    tf.config.experimental.enable_op_determinism()

    data = load(args.dataset)
    flights = sorted({int(f) for f in data["flight"]})
    per_flight = {f: flight_windows(data, f) for f in flights}
    sets = {name: [f for f in flights if split(f) == name] for name in ("train", "val", "test")}

    def stack(names: list[int]) -> tuple[np.ndarray, np.ndarray]:
        x = np.concatenate([per_flight[f][0] for f in names])
        y = np.concatenate([per_flight[f][1] for f in names])
        keep = y >= 0
        return x[keep], y[keep]

    x_train, y_train = stack(sets["train"])
    x_val, y_val = stack(sets["val"])
    x_test, y_test = stack(sets["test"])

    mean = x_train.mean(axis=0)
    std = np.maximum(x_train.std(axis=0), 1e-3)
    counts = np.bincount(y_train, minlength=len(CLASSES))
    class_weight = {i: float(len(y_train) / (len(CLASSES) * c)) for i, c in enumerate(counts)}

    model = build_model(F.FEATURES)
    model.compile(optimizer=tf.keras.optimizers.Adam(1e-3),
                  loss="sparse_categorical_crossentropy", metrics=["accuracy"])
    model.fit(
        (x_train - mean) / std, y_train,
        validation_data=((x_val - mean) / std, y_val),
        epochs=args.epochs, batch_size=128, class_weight=class_weight, verbose=2,
        callbacks=[tf.keras.callbacks.EarlyStopping(
            monitor="val_loss", patience=15, restore_best_weights=True)],
    )
    folded = fold_standardisation(model, mean, std)

    fp32_bytes = convert(folded, None)
    rng = np.random.default_rng(SEED)
    calibration = x_train[rng.choice(len(x_train), size=min(1000, len(x_train)), replace=False)]
    int8_bytes = convert(folded, calibration)
    fp32, int8 = TfliteModel(fp32_bytes), TfliteModel(int8_bytes)

    p_fp32 = fp32.probabilities(x_test)
    p_int8 = int8.probabilities(x_test)
    diff = np.abs(p_fp32 - p_int8)
    test_flights = [per_flight[f] for f in sets["test"]]
    soak = collect(SOAK_FLIGHTS, SOAK_FLIGHT_S, SOAK_SEED, fault_free=True)
    soak["meta"] = json.loads(str(soak["meta"]))
    soak_flights = [flight_windows(soak, f) for f in range(SOAK_FLIGHTS)]
    metrics = {
        "tensorflow": tf.__version__,
        "python": platform.python_version(),
        "parameters": int(folded.count_params()),
        "size": {"fp32": len(fp32_bytes), "int8": len(int8_bytes)},
        "data": {
            "flights": len(flights),
            "flight_s": float(np.round(len(data["t"]) / len(flights) / F.SAMPLE_RATE_HZ)),
            "seed": int(data.get("seed", np.array(1))),
            "sets": {
                name: {
                    "flights": len(members),
                    "windows": int(len(y)),
                    "per_class": np.bincount(y, minlength=len(CLASSES)).tolist(),
                }
                for name, members, y in (
                    ("training", sets["train"], y_train),
                    ("validation", sets["val"], y_val),
                    ("test", sets["test"], y_test),
                )
            },
        },
        "input_quantization": [float(v) for v in int8.input["quantization"]],
        "output_quantization": [float(v) for v in int8.output["quantization"]],
        "fp32": window_metrics(y_test, p_fp32),
        "int8": window_metrics(y_test, p_int8),
        "agreement": {
            "same_class": float((p_fp32.argmax(1) == p_int8.argmax(1)).mean()),
            "max_probability_diff": float(diff.max()),
            "mean_probability_diff": float(diff.mean()),
        },
        "alarm_fp32": alarm_metrics(test_flights, fp32),
        "alarm_int8": alarm_metrics(test_flights, int8),
        "soak_int8": alarm_metrics(soak_flights, int8),
        "soak_window_errors_int8": int(sum(
            int((int8.probabilities(x).argmax(axis=1) != 0).sum()) for x, *_ in soak_flights
        )),
        "soak_windows": int(sum(len(x) for x, *_ in soak_flights)),
    }

    MODELS.mkdir(parents=True, exist_ok=True)
    (MODELS / "vibration_fp32.tflite").write_bytes(fp32_bytes)
    (MODELS / "vibration_int8.tflite").write_bytes(int8_bytes)
    (MODELS / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    write_model_source(int8_bytes)
    write_vectors(data, sets["test"], int8)
    write_report(metrics)

    print(f"test accuracy: FP32 {pct(metrics['fp32']['accuracy'])}, "
          f"INT8 {pct(metrics['int8']['accuracy'])}; INT8 alarm: {metrics['alarm_int8']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
