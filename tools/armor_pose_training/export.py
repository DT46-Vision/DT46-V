import argparse
import hashlib
import json
import platform
import time
from pathlib import Path

import numpy as np
import onnx
import onnxruntime as ort
import torch

from .data import ArmorCrops, IMAGE_SIZE, TEAMS, read_manifest
from .evaluate import load_model
from .model import OUTPUT_NAMES


def main() -> None:
    parser = argparse.ArgumentParser(description="Export and check a CPU ONNX model on real validation crops")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cpu-threads", type=int, default=4)
    parser.add_argument("--repeats", type=int, default=100)
    args = parser.parse_args()
    if args.cpu_threads < 1 or args.repeats < 10:
        parser.error("Use positive thread count and at least ten timing repetitions")
    if args.output.exists() or args.output.with_suffix(".json").exists():
        raise ValueError("Export output already exists")
    torch.set_num_threads(args.cpu_threads)
    model, state = load_model(args.checkpoint, torch.device("cpu"))
    data = ArmorCrops(args.manifest, read_manifest(args.manifest), "validation", state["classes"])
    if len(data) < 4:
        raise ValueError("At least four validation crops are required")
    indices = np.linspace(0, len(data) - 1, min(32, len(data)), dtype=int)
    samples = [data[int(i)] for i in indices]
    images = torch.stack([s["image"] for s in samples])
    cameras = torch.stack([s["camera"] for s in samples])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    dynamic = {name: {0: "batch"} for name in ["image", "camera", *OUTPUT_NAMES]}
    with torch.inference_mode():
        torch.onnx.export(model, (images[:1], cameras[:1]), str(args.output),
                          input_names=["image", "camera"], output_names=OUTPUT_NAMES,
                          dynamic_axes=dynamic, opset_version=17, dynamo=False)
    onnx.checker.check_model(str(args.output))
    options = ort.SessionOptions()
    options.intra_op_num_threads = args.cpu_threads
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(args.output), sess_options=options, providers=["CPUExecutionProvider"])
    errors: dict[str, float] = {name: 0.0 for name in OUTPUT_NAMES}
    timings = {}
    with torch.inference_mode():
        for batch in (1, 2, 4):
            for offset in range(0, len(samples), batch):
                x, k = images[offset:offset + batch], cameras[offset:offset + batch]
                actual = session.run(None, {"image": x.numpy(), "camera": k.numpy()})
                expected = model(x, k)
                for name, a, e in zip(OUTPUT_NAMES, actual, expected):
                    np.testing.assert_allclose(a, e.numpy(), rtol=1e-4, atol=1e-4, err_msg=name)
                    errors[name] = max(errors[name], float(np.abs(a - e.numpy()).max()))
            feed = {"image": images[:batch].numpy(), "camera": cameras[:batch].numpy()}
            for _ in range(10):
                session.run(None, feed)
            elapsed = []
            for _ in range(args.repeats):
                start = time.perf_counter_ns()
                session.run(None, feed)
                elapsed.append((time.perf_counter_ns() - start) / 1e6)
            timings[str(batch)] = {"p50_ms": float(np.quantile(elapsed, .5)),
                                   "p95_ms": float(np.quantile(elapsed, .95)),
                                   "repeats": args.repeats}
    metadata = {
        "format_version": 1, "checkpoint_epoch": state["epoch"], "classes": state["classes"],
        "teams": TEAMS, "catalog": state["catalog"], "image_size": IMAGE_SIZE,
        "inputs": {"image": "float32 NCHW RGB in [0,1]; normalization is inside the model",
                   "camera": "float32 [log(fx/128),log(fy/128),cx/128,cy/128,k1,k2,p1,p2,k3]; crop K/D"},
        "outputs": OUTPUT_NAMES, "position_units": "metres",
        "camera_axes": "right/down/forward", "board_axes": "right/up/outward",
        "rotation_6d_order": "first column of R_camera_from_board, then second column",
        "decoding": "Apply Gram-Schmidt to two axes; reject nonfinite/zero/collinear axes or nonpositive depth. Range/bearing/incidence derive from the same p/R.",
        "presence_score": "sigmoid(armor_logit); uncalibrated; not a pose uncertainty estimate",
        "onnx_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "training_manifest_sha256": state["manifest_sha256"],
        "validation": {"onnx_checker": "passed", "cpu_provider": session.get_providers(),
                       "parity_crops": len(samples), "batch_sizes": [1, 2, 4],
                       "absolute_tolerance": 1e-4, "relative_tolerance": 1e-4,
                       "maximum_absolute_error": errors},
        "benchmark": {"scope": "ORT CPU inference only; prepared 128x128 crops. Excludes candidate extraction, crop/resize, preprocessing, pose decoding and IO.",
                      "cpu": platform.processor(), "threads": args.cpu_threads,
                      "onnxruntime": ort.__version__, "warmups": 10, "batches": timings},
    }
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(metadata, allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
