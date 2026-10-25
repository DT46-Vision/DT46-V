"""Adapt the simulation's camera/IMU/decision sample to DT46's existing pipeline."""

import numpy as np

from .transport import Connection


class RemoteCamera:
    def __init__(self, session: dict):
        self.connection = Connection.connect(session["port"])
        self.version = 0
        try:
            hello, _ = self.connection.request({"key": session["key"]})
            if hello.get("protocol") != 1:
                raise ValueError("Camera authentication/protocol failed")
            self.description, _ = self.connection.request({"op": "describe"})
            if self.description.get("protocol") != 1:
                raise ValueError("Unsupported camera protocol")
            self.parameters = self.description["camera"]
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        self.connection.close()

    def validate(self, **parameters) -> None:
        if "flip_image" in parameters:
            raise ValueError("Remove camera.flip_image; display rotation does not change algorithm input")
        self.connection.request({"op": "validate_camera", "parameters": parameters})

    def configure(self, **parameters) -> None:
        self.validate(**parameters)
        reply, _ = self.connection.request({"op": "configure_camera", "parameters": parameters})
        self.parameters = reply["parameters"]

    def next(self, status: dict, result: dict | None) -> dict | None:
        reply, pixels = self.connection.request({"op": "next", "after": self.version,
                                                 "status": status, "result": result})
        frame = reply["frame"]
        if frame is None:
            if pixels or reply["version"] != self.version:
                raise ValueError("Invalid empty camera response")
            return None
        width, height = frame["width"], frame["height"]
        if (not 0 < width <= 8192 or not 0 < height <= 8192 or frame["color_order"] != "RGB" or
                len(pixels) != width * height * 3 or reply["version"] <= self.version or
                frame["frame_number"] != reply["version"]):
            raise ValueError("Invalid RGB shape or frame identity")
        self.version = reply["version"]
        frame["image"] = np.frombuffer(pixels, dtype=np.uint8).reshape(height, width, 3)
        for key in ("K", "D", "optical", "muzzle_offset"):
            frame[key] = np.asarray(frame[key], dtype=np.float64)
        return frame


def aim_feedback(result: dict | None) -> dict | None:
    if result is None:
        return None
    fields = ("frame_number", "state_tick", "reset_generation", "simulation_time_s",
              "camera", "robot_id", "state", "tracked_id", "aim")
    return {"schema": 1, **{key: result[key] for key in fields}}


def process_frame(pipeline, frame: dict) -> dict:
    result = pipeline.process(frame["image"], frame["time"], frame["K"], frame["D"],
                              frame["optical"][:3, :3], frame["muzzle_offset"], frame["color"], frame["speed"])
    result.update(camera=frame["camera"], robot_id=frame["robot_id"], reset_generation=frame["generation"],
                  frame_number=frame["frame_number"], state_tick=frame["state_tick"], transport="virtual_camera",
                  sensor=frame["sensor"], parameter_revision=pipeline.revision,
                  bullet_speed_m_s=frame["speed"] or pipeline.nominal_speed,
                  bullet_speed_source="dual_photogate" if frame["speed"] else "tracker_config")
    for key in ("camera_ms", "camera_settings_version", "sampling_target_hz", "sampling_effective_limit_hz", "appearance"):
        if key in frame:
            result[key] = frame[key]
    result["input"]["optical_transform"] = frame["optical"].tolist()
    result["gimbal_control"] = {"yaw": result["yaw_error_deg"],
                                "pitch": result["pitch_error_deg"], "can_fire": int(result["can_fire"])}
    return result
