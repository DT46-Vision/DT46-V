"""NumPy interface to the original DT46 C++ detector, PnP and tracker."""

import ctypes as ct
import json
import os
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
STATES = ("LOST", "DETECTING", "TRACKING", "TEMP_LOST")


class Observation(ct.Structure):
    _fields_ = [("id", ct.c_int), ("confidence", ct.c_double),
                ("position_camera_mm", ct.c_double*3), ("yaw_deg", ct.c_double),
                ("corners", ct.c_double*8)]


class Result(ct.Structure):
    _fields_ = [(name, ct.c_int) for name in ("count", "light_count", "state", "tracked_id", "can_fire")]
    _fields_ += [("yaw_error_deg", ct.c_double), ("pitch_error_deg", ct.c_double),
                 ("position", ct.c_double*3), ("velocity", ct.c_double*3),
                 ("observations", Observation*64)]


def load_parameters(package: str, filename: str) -> dict:
    source = ROOT/"src"/package/"config"/filename
    return yaml.safe_load(source.read_text(encoding="utf-8"))[package]["ros__parameters"]


class AimPipeline:
    def __init__(self, work_dir: Path | str | None = None):
        self.handle = None
        self.dll_directories = []
        self.work = Path(work_dir) if work_dir else ROOT/".cache/native"
        extension = ".dll" if os.name == "nt" else ".so"
        library = self.work/"build"/f"dt46_aim{extension}"
        if not library.exists():
            raise FileNotFoundError(f"Build the native library first: python {ROOT/'standalone/build.py'}")
        if os.name == "nt":
            folders = {library.parent.resolve(), Path("D:/msys2/ucrt64/bin")}
            folders.update(p.parent.resolve() for p in (self.work/"opencv").rglob("*opencv_core*.dll"))
            self.dll_directories = [os.add_dll_directory(str(p)) for p in folders if p.is_dir()]
        self.library = ct.CDLL(str(library.resolve()))
        self.library.dt46_result_size.restype = ct.c_int
        if self.library.dt46_result_size() != ct.sizeof(Result):
            raise RuntimeError("Native result layout changed; rebuild the native library")
        self.library.dt46_error.restype = ct.c_char_p
        self.library.dt46_create.argtypes = [ct.c_char_p, ct.c_char_p]
        self.library.dt46_create.restype = ct.c_void_p
        self.library.dt46_destroy.argtypes = [ct.c_void_p]
        self.library.dt46_destroy.restype = None
        double = ct.POINTER(ct.c_double)
        self.library.dt46_process.argtypes = [ct.c_void_p, ct.POINTER(ct.c_uint8), ct.c_int, ct.c_int,
            ct.c_double, double, double, double, double, ct.c_int, ct.c_double, ct.POINTER(Result)]
        self.library.dt46_process.restype = ct.c_int
        detector = load_parameters("rm_detector", "detector_params_infantry.yaml")
        tracker = load_parameters("rm_tracker", "tracker_params_infantry.yaml")
        self.nominal_speed = float(tracker.get("bullet_speed", 28.0))
        self.model = ROOT/"src/rm_detector/model"/detector["cls_model_file"]
        numeric = lambda parameters: {k: int(v) if isinstance(v, bool) else v
                                     for k, v in parameters.items() if isinstance(v, (int, float))}
        self.parameters = json.dumps({"detector": numeric(detector), "tracker": numeric(tracker)}).encode()
        self.reset()

    def error(self) -> RuntimeError:
        return RuntimeError(self.library.dt46_error().decode("utf-8", errors="replace"))

    def reset(self) -> None:
        self.close_context()
        self.handle = self.library.dt46_create(str(self.model).encode("utf-8"), self.parameters)
        if not self.handle:
            raise self.error()

    def close_context(self) -> None:
        if self.handle:
            self.library.dt46_destroy(self.handle)
            self.handle = None

    def close(self) -> None:
        self.close_context()
        for directory in self.dll_directories:
            directory.close()
        self.dll_directories = []

    def process(self, rgb: np.ndarray, timestamp: float, K: np.ndarray, D: np.ndarray,
                optical_rotation: np.ndarray, muzzle_offset: np.ndarray,
                target_color: int, measured_speed: float = 0.0) -> dict:
        if not self.handle:
            raise RuntimeError("The aim pipeline is closed")
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("Expected an HxWx3 uint8 RGB image")
        if target_color not in (0, 1) or not np.isfinite(timestamp) or timestamp < 0:
            raise ValueError("Invalid target colour or simulation timestamp")
        arrays = [np.ascontiguousarray(value, dtype=np.float64) for value in (K, D, optical_rotation, muzzle_offset)]
        for array, shape in zip(arrays, ((3, 3), (5,), (3, 3), (3,))):
            if array.shape != shape or not np.isfinite(array).all():
                raise ValueError(f"Expected finite input with shape {shape}")
        if arrays[0][0, 0] <= 0 or arrays[0][1, 1] <= 0 or not np.isfinite(measured_speed) or measured_speed < 0:
            raise ValueError("Invalid camera focal length or bullet speed")
        rgb = np.ascontiguousarray(rgb)
        result = Result()
        code = self.library.dt46_process(self.handle, rgb.ctypes.data_as(ct.POINTER(ct.c_uint8)),
            rgb.shape[1], rgb.shape[0], timestamp,
            *(a.ctypes.data_as(ct.POINTER(ct.c_double)) for a in arrays),
            target_color, measured_speed, ct.byref(result))
        if code:
            raise self.error()
        observations = [{"armor_id": a.id, "classifier_score": a.confidence,
            "position_camera_m": (np.array(a.position_camera_mm)/1000).tolist(),
            "yaw_deg": a.yaw_deg, "corners_px": np.array(a.corners).reshape(4, 2).tolist()}
            for a in result.observations[:result.count]]
        return {"simulation_time_s": timestamp, "state": STATES[result.state],
                "tracked_id": result.tracked_id, "can_fire": bool(result.can_fire),
                "yaw_error_deg": result.yaw_error_deg, "pitch_error_deg": result.pitch_error_deg,
                "position_camera_origin_world_axes_m": list(result.position),
                "velocity_world_axes_m_s": list(result.velocity),
                "light_count": result.light_count, "observations": observations}

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
