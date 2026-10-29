"""Replay saved RGB frames through the unchanged C ABI (including JSON snapshot generation)."""
import argparse
import ctypes as c
import hashlib
import json
import os
from pathlib import Path
import time

import numpy as np
from PIL import Image


class Observation(c.Structure):
    _fields_ = [("id", c.c_int), ("confidence", c.c_double), ("position", c.c_double * 3),
                ("yaw", c.c_double), ("corners", c.c_double * 8)]


class Result(c.Structure):
    _fields_ = [(name, c.c_int) for name in ("count", "lights", "state", "id", "fire", "valid")] + [
        (name, c.c_double) for name in ("yaw", "pitch", "yaw_error", "pitch_error")] + [
        ("position", c.c_double * 3), ("velocity", c.c_double * 3), ("observations", Observation * 64)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("dll", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--iterations", type=int, default=240)
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    # Do not add every PATH directory: unrelated SDKs ship incompatible GCC DLLs.
    handles = [os.add_dll_directory(str(Path(p).resolve())) for p in os.environ["PATH"].split(os.pathsep)
               if p and ("opencv" in p.lower() or "ucrt64" in p.lower()) and Path(p).is_dir()]
    libraries = []
    for label, dll in ([("before", args.baseline)] if args.baseline else []) + [("after", args.dll)]:
        lib = c.CDLL(str(dll.resolve()))
        lib.dt46_create.argtypes = [c.c_char_p, c.c_char_p]
        lib.dt46_create.restype = c.c_void_p
        lib.dt46_destroy.argtypes = [c.c_void_p]
        lib.dt46_error.restype = c.c_char_p
        lib.dt46_snapshot.argtypes = [c.c_void_p]
        lib.dt46_snapshot.restype = c.c_char_p
        stage = getattr(lib, "dt46_stage_times", None)
        if stage:
            stage.argtypes = [c.c_void_p, c.POINTER(c.c_double)]
        double_ptr = c.POINTER(c.c_double)
        lib.dt46_process.argtypes = [c.c_void_p, c.c_void_p, c.c_int, c.c_int, c.c_double,
                                    double_ptr, double_ptr, double_ptr, double_ptr, c.c_int,
                                    c.c_double, c.POINTER(Result)]
        assert lib.dt46_result_size() == c.sizeof(Result)
        libraries.append((label, dll, lib, stage))
    folders = [root / "outputs/vision_tracking_20261030" / name for name in
               ("final-20261030-013123", "final-motion2-20261030-013028")]
    rows = []
    for folder in folders:
        for frame in sorted(folder.glob("*/vision/frame_g*/result.json")):
            inp = json.loads(frame.read_text())["input"]
            params = json.loads(frame.with_name("parameters.json").read_text())
            for group in ("tracker", "detector"):
                params[group] = {k: int(v) if isinstance(v, bool) else v for k, v in params[group].items()}
            image = np.array(Image.open(frame.with_name("image_raw.png")).convert("RGB"))
            arrays = [np.asarray(inp[key], dtype=np.float64).ravel() for key in
                      ("K", "D", "optical_rotation", "muzzle_offset_camera_m")]
            pointers = [a.ctypes.data_as(double_ptr) for a in arrays]
            for mode in ("fixed", "changing"):
                contexts = []
                for label, dll, lib, stage in libraries:
                    t0 = time.perf_counter_ns()
                    ctx = lib.dt46_create(str(root / "src/rm_detector/model" / params["model"]).encode(),
                                         json.dumps(params).encode())
                    startup_ms = (time.perf_counter_ns() - t0) / 1e6
                    if not ctx:
                        raise RuntimeError(lib.dt46_error().decode())
                    contexts.append(dict(label=label, lib=lib, stage=stage, ctx=ctx, result=Result(),
                                         startup_ms=startup_ms, times=[], stages=[]))
                stage_values = (c.c_double * 5)()
                for i in range(40 + args.iterations):
                    speed = 25.0 if mode == "fixed" else (24.7 if i % 2 else 25.3)
                    # Same input, interleaved execution and alternating order reduce machine-load bias.
                    for record in contexts[::1 if i % 2 else -1]:
                        lib, ctx, result = record["lib"], record["ctx"], record["result"]
                        start = time.perf_counter_ns()
                        status = lib.dt46_process(ctx, image.ctypes.data, image.shape[1], image.shape[0],
                                                 1.0 + i * .01, *pointers, inp["target_color"], speed, c.byref(result))
                        elapsed = (time.perf_counter_ns() - start) / 1e6
                        if status:
                            raise RuntimeError(lib.dt46_error().decode())
                        if i >= 40:
                            record["times"].append(elapsed)
                            if record["stage"]:
                                record["stage"](ctx, stage_values)
                                record["stages"].append(list(stage_values))
                for record in contexts:
                    result, timings = record["result"], record["times"]
                    row = {"frame": str(frame.relative_to(root)), "mode": mode, "variant": record["label"],
                           "startup_ms": record["startup_ms"], "median_ms": float(np.median(timings)),
                           "p95_ms": float(np.percentile(timings, 95)), "p99_ms": float(np.percentile(timings, 99)),
                           "max_ms": max(timings), "count": result.count, "lights": result.lights, "state": result.state,
                           "yaw_deg": result.yaw, "pitch_deg": result.pitch, "fire": result.fire,
                           "observations": [{"id": a.id, "confidence": a.confidence, "position": list(a.position), "corners": list(a.corners)}
                                            for a in result.observations[:result.count]], "samples_ms": timings}
                    rows.append(row)
                    if record["stages"]:
                        row["stage_median_ms"] = dict(zip(("preparation", "detector", "pnp", "tracker", "snapshot"),
                                                          np.median(record["stages"], axis=0)))
                    record["lib"].dt46_destroy(record["ctx"])
                    print(frame.parent.parent.parent.name, mode, record["label"], round(row["median_ms"], 3), round(row["p95_ms"], 3), flush=True)
    args.output.write_text(json.dumps({"dll_sha256": {label: hashlib.sha256(dll.read_bytes()).hexdigest() for label, dll, _, _ in libraries},
                                      "iterations": args.iterations, "rows": rows}, indent=2))


if __name__ == "__main__":
    main()
