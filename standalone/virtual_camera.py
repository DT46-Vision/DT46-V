"""Run the non-ROS DT46-V project against a local virtual-camera endpoint."""

import argparse
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace
from multiprocessing.connection import Client

import numpy as np
from PIL import Image

from .aim import AimPipeline
from .debug_window import DebugControl
from .vision_debug import VisionDebug


def process_frame(pipeline, frame):
    """Run only the original DT46 vision core; this function cannot actuate a car."""
    started = time.perf_counter()
    result = pipeline.process(frame["image"], frame["time"], frame["K"], frame["D"],
                              frame["optical"][:3, :3], frame["muzzle_offset"], frame["color"], frame["speed"])
    result.update(camera=frame["camera"], robot_id=frame["robot_id"], detect_color=frame["color"],
                  reset_generation=frame["generation"], transport="native_in_process",
                  bullet_speed_m_s=frame["speed"] or pipeline.nominal_speed,
                  bullet_speed_source="dual_photogate" if frame["speed"] else "tracker_config",
                  camera_ms=frame["camera_ms"], processing_ms=(time.perf_counter()-started)*1000)
    result["input"]["optical_transform"] = frame["optical"].tolist()
    result["sensor"] = frame["sensor"]
    return result



class RemoteCamera:
    def __init__(self, connection, parameters: dict):
        self.connection, self.parameters = connection, parameters

    def request(self, operation: str, **values) -> dict:
        self.connection.send({"op": operation, **values})
        reply = self.connection.recv()
        if "error" in reply:
            raise ValueError(reply["error"])
        return reply

    def validate(self, **parameters) -> None:
        self.request("validate_camera", parameters=parameters)

    def configure(self, **parameters) -> None:
        self.parameters = self.request("configure_camera", parameters=parameters)["parameters"]


def run(args) -> None:
    args.output.mkdir(parents=True, exist_ok=True)
    frames, displays, version, last_time = 0, 0, 0, -1.
    ui, last_packet, last_result = None, None, None
    events = []
    with Client(("127.0.0.1", args.port), authkey=bytes.fromhex(args.key)) as connection:
        sensor = RemoteCamera(connection, {})
        description = sensor.request("describe")
        if description["protocol"] != 1:
            raise ValueError("Unsupported virtual-camera protocol")
        sensor.parameters = description["camera"]
        with AimPipeline(args.work_dir, args.profile, debug=True) as pipeline:
            pipeline.configure(tracker=description["physics"])
            aim = SimpleNamespace(pipeline=pipeline, camera="onboard", last_time=None,
                                  stage=SimpleNamespace(camera_sensors={"onboard": sensor}))
            last_display, generation = 0., -1
            debug = DebugControl()
            try:
                with (args.output/"frames.jsonl").open("w", encoding="utf-8") as log:
                    while True:
                        status = {"pid": os.getpid(), "frames": frames, "debug_open": ui is not None,
                                  "context": pipeline.handle, "state": last_result["state"] if last_result else "STARTING"}
                        reply = sensor.request("next", after=version, status=status)
                        version = reply["version"]
                        if debug.requested.is_set():
                            debug.requested.clear()
                            if ui is None:
                                ui = VisionDebug(aim, args.output)
                                events.append({"event": "open", "frames": frames, "context": pipeline.handle})
                                if last_packet is not None:
                                    ui.update(last_packet["image"], last_result)
                            else:
                                ui.window.deiconify()
                                ui.window.lift()
                        if ui is not None:
                            ui.pump()
                            if not ui.running:
                                ui.window.destroy()
                                ui = None
                                events.append({"event": "close", "frames": frames, "context": pipeline.handle})
                        packet = reply["frame"]
                        if packet is not None and packet["generation"] != generation:
                            if generation != -1:
                                pipeline.reset()
                            generation, last_time = packet["generation"], -1.
                        if packet is None or packet["time"] <= last_time:
                            time.sleep(.001)
                            continue
                        result = process_frame(pipeline, packet)
                        result.update(transport="virtual_camera", frame_number=packet["frame_number"], state_tick=packet["state_tick"])
                        frames += 1
                        last_time, last_packet, last_result = packet["time"], packet, result
                        log.write(json.dumps(result)+"\n")
                        if frames == 1:
                            Image.fromarray(packet["image"]).save(args.output/"input.png")
                        if ui is not None and time.perf_counter()-last_display >= 1/30:
                            last_display = time.perf_counter()
                            ui.update(packet["image"], result)
                            displays += 1
            except (EOFError, ConnectionError, OSError):
                pass
            finally:
                debug.close()
                if ui is not None:
                    ui.close()
                    ui.window.destroy()
                report = {"pid": os.getpid(), "frames": frames, "display_frames": displays,
                          "debug_events": events, "last_frame": last_result}
                (args.output/"report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--key", required=True)
    parser.add_argument("--work-dir", type=Path, default=Path(__file__).resolve().parents[1]/".cache/native")
    parser.add_argument("--profile", choices=("infantry", "sentry", "hero"), default="infantry")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1]/".cache/virtual_camera")
    run(parser.parse_args())


if __name__ == "__main__":
    main()
