"""NumPy interface to the original DT46 C++ detector, PnP and tracker."""

import ctypes as ct
import json
import os
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
STATES = ("LOST", "DETECTING", "TRACKING", "TEMP_LOST")
DEBUG_IMAGES = ("result", "crop_img", "bin_img", "img_armor", "img_armor_processed", "img_debug", "gray")
DETECTOR_PARAMETERS = ("light_area_min", "light_h_w_ratio", "light_angle_min", "light_angle_max",
    "light_red_ratio", "light_blue_ratio", "height_rate_tol", "height_multiplier_min",
    "height_multiplier_max", "row_stretch", "col_stretch", "roi_crop", "roi_scale", "binary_val", "display_mode")
TRACKER_PARAMETERS = ("max_match_distance", "max_match_yaw_diff", "jump_cooldown_max", "tracking_thres",
    "lost_thres", "dist_tol", "ekf_QR_q_xyz", "ekf_QR_q_yaw", "ekf_QR_q_r", "ekf_QR_r_xyz_factor",
    "ekf_QR_r_yaw", "ekf_QR_stable_dist", "radius_r_min", "radius_r_max", "system_delay", "controller_delay",
    "bullet_speed", "k_v2", "gravity", "shootable_dist", "yaw_tolerance_deg", "pitch_tolerance_deg",
    "min_spinning_vel", "spin_enter_time", "spin_exit_time", "cam_to_gun_rpy_r", "cam_to_gun_rpy_p", "cam_to_gun_rpy_y")


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
    def __init__(self, work_dir: Path | str | None = None, profile: str = "infantry", debug: bool = False):
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
        self.library.dt46_snapshot.argtypes = [ct.c_void_p]
        self.library.dt46_snapshot.restype = ct.c_char_p
        self.library.dt46_debug_image.argtypes = [ct.c_void_p, ct.c_int, ct.POINTER(ct.c_uint8), ct.c_int,
                                                ct.POINTER(ct.c_int), ct.POINTER(ct.c_int), ct.POINTER(ct.c_int)]
        self.library.dt46_debug_image.restype = ct.c_int
        self.library.dt46_create.argtypes = [ct.c_char_p, ct.c_char_p]
        self.library.dt46_create.restype = ct.c_void_p
        self.library.dt46_destroy.argtypes = [ct.c_void_p]
        self.library.dt46_destroy.restype = None
        double = ct.POINTER(ct.c_double)
        self.library.dt46_process.argtypes = [ct.c_void_p, ct.POINTER(ct.c_uint8), ct.c_int, ct.c_int,
            ct.c_double, double, double, double, double, ct.c_int, ct.c_double, ct.POINTER(Result)]
        self.library.dt46_process.restype = ct.c_int
        if profile not in ("infantry", "sentry", "hero"):
            raise ValueError("Select infantry, sentry or hero parameters")
        detector = load_parameters("rm_detector", f"detector_params_{profile}.yaml")
        tracker = load_parameters("rm_tracker", f"tracker_params_{profile}.yaml")
        tracker = {"controller_delay": .02, "k_v2": .019, "gravity": 9.81,
                   "spin_enter_time": .3, "spin_exit_time": .15, **tracker}
        self.settings = {"detector": {k: detector[k] for k in DETECTOR_PARAMETERS if k in detector},
                         "tracker": {k: tracker[k] for k in TRACKER_PARAMETERS if k in tracker},
                         "model": detector["cls_model_file"], "profile": profile}
        self.settings["detector"]["display_mode"] = debug
        self.reset()

    def error(self) -> RuntimeError:
        return RuntimeError(self.library.dt46_error().decode("utf-8", errors="replace"))

    def reset(self) -> None:
        numeric = lambda values: {k: int(v) if isinstance(v, bool) else v for k, v in values.items()}
        parameters = json.dumps({name: numeric(self.settings[name]) for name in ("detector", "tracker")}).encode()
        model = ROOT/"src/rm_detector/model"/self.settings["model"]
        handle = self.library.dt46_create(str(model).encode("utf-8"), parameters)
        if not handle:
            raise self.error()
        self.close_context()
        self.handle = handle
        self.nominal_speed = float(self.settings["tracker"]["bullet_speed"])

    def configure(self, *, detector: dict | None = None, tracker: dict | None = None,
                  model: str | None = None) -> None:
        previous = self.settings
        updated = {**previous, "detector": {**previous["detector"]}, "tracker": {**previous["tracker"]}}
        for name, changes, allowed in (("detector", detector or {}, DETECTOR_PARAMETERS),
                                       ("tracker", tracker or {}, TRACKER_PARAMETERS)):
            for key, value in changes.items():
                if key not in allowed or not isinstance(value, (int, float)) or not np.isfinite(value):
                    raise ValueError(f"Invalid {name} parameter {key}")
                updated[name][key] = value
        if model is not None:
            updated["model"] = model
        if not 0 < updated["detector"]["roi_scale"] <= 1 or not 0 <= updated["detector"]["binary_val"] <= 255:
            raise ValueError("ROI scale must be (0,1]; binary threshold must be [0,255]")
        for key in ("bullet_speed", "gravity"):
            if updated["tracker"][key] <= 0:
                raise ValueError(f"{key} must be positive")
        if updated["tracker"]["k_v2"] < 0:
            raise ValueError("Drag must be nonnegative")
        d, t = updated["detector"], updated["tracker"]
        for key in ("row_stretch", "col_stretch", "light_h_w_ratio", "height_rate_tol"):
            if d[key] <= 0:
                raise ValueError(f"{key} must be positive")
        if d["light_area_min"] < 0 or d["light_angle_min"] > d["light_angle_max"]:
            raise ValueError("Invalid light area or angle bounds")
        if d["height_multiplier_min"] < 0 or d["height_multiplier_min"] > d["height_multiplier_max"]:
            raise ValueError("Invalid armor height bounds")
        if t["radius_r_min"] <= 0 or t["radius_r_min"] > t["radius_r_max"]:
            raise ValueError("Invalid tracker radius bounds")
        for key in ("tracking_thres", "lost_thres", "jump_cooldown_max"):
            if t[key] < 0 or t[key] != int(t[key]):
                raise ValueError(f"{key} must be a nonnegative integer")
        for key in ("system_delay", "controller_delay", "spin_enter_time", "spin_exit_time"):
            if t[key] < 0:
                raise ValueError(f"{key} must be nonnegative")
        self.settings = updated
        try:
            self.reset()
        except Exception:
            self.settings = previous
            raise

    def debug_image(self, name: str) -> np.ndarray | None:
        if not self.handle:
            raise RuntimeError("The aim pipeline is closed")
        width, height, channels = ct.c_int(), ct.c_int(), ct.c_int()
        args = (self.handle, DEBUG_IMAGES.index(name))
        dimensions = (ct.byref(width), ct.byref(height), ct.byref(channels))
        size = self.library.dt46_debug_image(*args, None, 0, *dimensions)
        if size < 0:
            raise self.error()
        if not size:
            return None
        output = np.empty((height.value, width.value, channels.value), dtype=np.uint8)
        if self.library.dt46_debug_image(*args, output.ctypes.data_as(ct.POINTER(ct.c_uint8)), size, *dimensions) < 0:
            raise self.error()
        return output[:, :, 0] if channels.value == 1 else output

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
                "light_count": result.light_count, "observations": observations,
                "tracker_debug": json.loads(self.library.dt46_snapshot(self.handle)),
                "input": {"K": arrays[0].tolist(), "D": arrays[1].tolist(),
                          "optical_rotation": arrays[2].tolist(), "muzzle_offset_camera_m": arrays[3].tolist()}}

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
