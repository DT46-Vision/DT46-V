"""Python launch and lifecycle for DT46's Windows simulation adapter."""

import argparse
from contextlib import ExitStack
import json
import math
import os
from pathlib import Path
import queue
import time
from types import SimpleNamespace

import cv2
from PIL import Image

from .aim import AimPipeline, ROOT
from .camera import RemoteCamera, aim_feedback, process_frame
from .debug import ballistic_image
from .debug_window import DebugControl, SESSION, command
from .transport import ConnectionClosed


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


class Session:
    def __init__(self, args, resources: ExitStack):
        self.args = args
        self.frames, self.displays, self.feedback_sent, self.video_frames = 0, 0, 0, 0
        self.generation, self.last_time, self.result, self.packet = None, -1.0, None, None
        self.result_revision, self.logged_revision, self.logged_camera = 0, -1, -1
        self.ui, self.writer, self.last_record, self.last_draw = None, None, -math.inf, 0.0
        self.began = time.perf_counter()
        self.log = resources.enter_context((args.output / "frames.jsonl").open("w", encoding="utf-8"))
        self.events = resources.enter_context((args.output / "debug_events.jsonl").open("w", encoding="utf-8"))
        self.revisions = resources.enter_context((args.output / "parameter_revisions.jsonl").open("w", encoding="utf-8"))
        self.camera = RemoteCamera(read_json(args.camera_session))
        resources.callback(self.camera.close)
        self.pipeline = AimPipeline(args.work_dir, args.profile, debug=True, library=args.library, dll_dirs=tuple(args.dll_dir))
        resources.callback(self.pipeline.close)
        self.control = DebugControl(args.debug_session)
        resources.callback(self.control.close)
        resources.callback(self.close_debug)
        resources.callback(self.close_recording)
        settings = read_json(args.parameters) if args.parameters else {}
        self.apply_parameters(settings or read_json(ROOT / "standalone/virtual_camera.json"))
        if args.debug:
            self.open_debug()

    def event(self, name: str, **values) -> None:
        self.events.write(json.dumps({"event": name, "frames": self.frames, "context": self.pipeline.handle,
                                      "revision": self.pipeline.revision, **values}, allow_nan=False) + "\n")
        self.events.flush()

    def parameters(self) -> dict:
        return {"camera": dict(self.camera.parameters), "detector": dict(self.pipeline.settings["detector"]),
                "tracker": dict(self.pipeline.settings["tracker"]), "model": self.pipeline.settings["model"]}

    def apply_parameters(self, settings: dict) -> None:
        if not isinstance(settings, dict) or set(settings) - {"camera", "detector", "tracker", "model"}:
            raise ValueError("Expected camera/detector/tracker/model parameter groups")
        if any(not isinstance(settings[name], dict) for name in ("camera", "detector", "tracker") if name in settings):
            raise ValueError("Parameter groups must be objects")
        camera = settings.get("camera")
        if camera is not None:
            self.camera.validate(**camera)
        tracker = dict(settings.get("tracker", {}))
        for key, value in self.camera.description["physics"].items():
            if key in tracker and tracker[key] != value:
                raise ValueError(f"{key} follows simulation physics ({value})")
            tracker[key] = value
        self.pipeline.configure(detector=settings.get("detector"), tracker=tracker, model=settings.get("model"))
        self.result, self.packet, self.result_revision = None, None, 0
        if camera is not None:
            self.camera.configure(**camera)
        if self.ui:
            self.ui.last_frame = None
            self.ui.refresh_parameters()
        self.event("configured", parameters=self.parameters())

    def open_debug(self) -> None:
        if self.ui is None:
            from .vision_debug import VisionDebug
            aim = SimpleNamespace(pipeline=self.pipeline, camera="onboard", apply_parameters=self.apply_parameters,
                                  parameters=self.parameters, stage=SimpleNamespace(camera_sensors={"onboard": self.camera}))
            self.ui = VisionDebug(aim, self.args.output, hidden=self.args.debug_hidden)
            if self.packet is not None:
                self.ui.update(self.packet["image"], self.result)
            self.event("opened")
        elif not self.args.debug_hidden:
            self.ui.window.deiconify()
            self.ui.window.lift()

    def close_debug(self) -> None:
        if self.ui:
            self.ui.window.destroy()
            self.ui = None
            self.event("closed")

    def debug_commands(self) -> None:
        while True:
            try:
                request = self.control.commands.get_nowait()
            except queue.Empty:
                break
            operation = request["op"]
            try:
                if operation == "show":
                    self.open_debug()
                elif operation == "close":
                    self.close_debug()
                elif operation == "save":
                    if self.ui is None or self.ui.last_frame is None:
                        raise ValueError("Open debug and wait for a current frame before saving")
                    folder = self.ui.save_frame()
                    self.event("saved", path=str(folder))
                elif operation == "reset":
                    self.pipeline.reset()
                    self.result, self.packet, self.result_revision = None, None, 0
                    if self.ui:
                        self.ui.last_frame = None
                    self.event("reset")
                elif operation in ("configure", "load_parameters"):
                    settings = request["parameters"] if operation == "configure" else read_json(Path(request["path"]))
                    self.apply_parameters(settings)
                elif operation == "save_parameters":
                    Path(request["path"]).write_text(json.dumps(self.parameters(), indent=2) + "\n", encoding="utf-8")
                    self.event("parameters_saved", path=request["path"])
                self.event("command_done", operation=operation, request_id=request.get("request_id"), error="")
            except (ValueError, RuntimeError, OSError, TypeError) as error:
                self.event("command_done", operation=operation, request_id=request.get("request_id"), error=str(error))

    def status(self) -> dict:
        return {"pid": os.getpid(), "frames": self.frames, "context": self.pipeline.handle,
                "parameter_revision": self.pipeline.revision, "reset_generation": self.generation,
                "state": self.result["state"] if self.result else "STARTING", "debug_open": self.ui is not None,
                "parameters": self.parameters()}

    def process(self, packet: dict) -> None:
        generation = packet["generation"]
        if generation != self.generation:
            if self.generation is not None:
                self.pipeline.reset()
                self.event("generation_reset", generation=generation)
            self.generation, self.last_time = generation, -1.0
        if packet["time"] <= self.last_time:
            self.result, self.result_revision = None, 0
            return
        started = time.perf_counter()
        self.result = process_frame(self.pipeline, packet)
        self.result["processing_ms"] = (time.perf_counter() - started) * 1000
        self.result_revision, self.packet, self.last_time = self.pipeline.revision, packet, packet["time"]
        self.frames += 1
        self.log.write(json.dumps(self.result, allow_nan=False) + "\n")
        self.log.flush()
        camera_version = packet.get("camera_settings_version", 0)
        if self.logged_revision != self.result_revision or self.logged_camera != camera_version:
            revision = {key: self.result[key] for key in ("parameter_revision", "reset_generation", "frame_number", "state_tick", "simulation_time_s")}
            revision.update(parameters=self.parameters(), camera_settings_version=camera_version, sensor=packet["sensor"])
            self.revisions.write(json.dumps(revision, allow_nan=False) + "\n")
            self.revisions.flush()
            self.logged_revision, self.logged_camera = self.result_revision, camera_version
        if self.frames == 1:
            Image.fromarray(packet["image"]).save(self.args.output / "input.png")
        self.record(packet)

    def record(self, packet: dict) -> None:
        if not self.args.record or not self.pipeline.settings["tracker"]["display"]:
            return
        now = time.perf_counter()
        if now - self.last_record < 1 / self.args.video_fps:
            return
        pixels = ballistic_image(packet["image"], self.result, self.pipeline.settings["tracker"]["text_size"])
        if self.writer is None:
            self.args.record.parent.mkdir(parents=True, exist_ok=True)
            self.writer = cv2.VideoWriter(str(self.args.record), cv2.VideoWriter_fourcc(*"mp4v"),
                                          self.args.video_fps, (pixels.shape[1], pixels.shape[0]))
            if not self.writer.isOpened():
                raise RuntimeError("Cannot open video writer")
        self.writer.write(cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))
        self.video_frames += 1
        self.last_record = now

    def close_recording(self) -> None:
        if self.writer:
            self.writer.release()
            self.writer = None

    def run(self) -> None:
        while (not self.args.duration or time.perf_counter() - self.began < self.args.duration) and (not self.args.frames or self.frames < self.args.frames):
            self.debug_commands()
            state = self.status()
            self.control.publish(state)
            result = aim_feedback(self.result) if self.result_revision == self.pipeline.revision else None
            packet = self.camera.next(state, result)
            if result:
                self.feedback_sent += 1
            if packet is not None:
                self.process(packet)
            else:
                time.sleep(0.001)
            if self.ui:
                self.ui.pump()
                if not self.ui.running:
                    self.close_debug()
                elif self.packet is not None and time.perf_counter() - self.last_draw >= 1/30:
                    self.ui.update(self.packet["image"], self.result)
                    self.displays += 1
                    self.last_draw = time.perf_counter()


def run(args) -> int:
    args.output.mkdir(parents=True, exist_ok=True)
    session, error, stopped = None, "", "limit"
    began = time.perf_counter()
    try:
        with ExitStack() as resources:
            session = Session(args, resources)
            session.run()
    except (ConnectionClosed, ConnectionResetError, ConnectionAbortedError, BrokenPipeError) as failure:
        stopped = str(failure)
    except Exception as failure:
        stopped, error = "error", f"{type(failure).__name__}: {failure}"
    if session is None or not session.frames:
        error = error or "No frames processed"
    report = {"error": error, "stopped": stopped, "wall_seconds": time.perf_counter()-began,
              "frames": session.frames if session else 0, "profile": args.profile,
              "display_frames": session.displays if session else 0,
              "feedback_requests": session.feedback_sent if session else 0,
              "video_frames": session.video_frames if session else 0,
              "last_frame": session.result if session else None}
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("frames", "error", "stopped")}))
    return int(bool(error))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--camera-session", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/virtual_camera")
    parser.add_argument("--debug-session", type=Path, default=SESSION)
    parser.add_argument("--parameters", type=Path)
    parser.add_argument("--profile", choices=("infantry", "sentry", "hero"), default="infantry")
    parser.add_argument("--library", type=Path)
    parser.add_argument("--dll-dir", type=Path, action="append", default=[])
    parser.add_argument("--work-dir", type=Path)
    parser.add_argument("--duration", type=float, default=0)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--video-fps", type=float, default=20)
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--debug-hidden", action="store_true")
    commands = parser.add_mutually_exclusive_group()
    commands.add_argument("--show-debug", action="store_true")
    commands.add_argument("--close-debug", action="store_true")
    commands.add_argument("--save-debug", action="store_true")
    args = parser.parse_args()
    for flag, operation in ((args.show_debug, "show"), (args.close_debug, "close"), (args.save_debug, "save")):
        if flag:
            print(json.dumps(command(args.debug_session, operation)))
            return 0
    if args.root.resolve() != ROOT:
        parser.error("--root must identify the DT46 checkout containing this installed Python package")
    if args.camera_session is None:
        parser.error("--camera-session is required")
    if not math.isfinite(args.duration) or args.duration < 0 or args.frames < 0 or not math.isfinite(args.video_fps) or args.video_fps <= 0:
        parser.error("Invalid duration, frame limit or video rate")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
