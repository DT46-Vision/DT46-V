"""Exercise the actual Python session and Tk callbacks against an unchanged simulator."""

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import subprocess
import shutil
import time
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

from rm_vision_windows.aim import ROOT
from rm_vision_windows.debug_window import command
from rm_vision_windows.virtual_camera import Session


def read(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulator", type=Path, required=True)
    parser.add_argument("--sim-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args()
    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    checks = {"passed": False, "simulator": str(options.simulator.resolve()), "checks": []}

    def require(condition, name):
        if not condition:
            raise AssertionError(name)
        checks["checks"].append(name)

    try:
        with ExitStack() as resources:
            log = resources.enter_context((output / "simulator.log").open("w", encoding="utf-8"))
            simulator = subprocess.Popen([str(options.simulator.resolve()), "--root", str(options.sim_root.resolve()),
                "--headless-drive", "--no-vision", "--duration", "60", "--width", "640", "--height", "512",
                "--output", str(output / "simulation")], stdout=log, stderr=subprocess.STDOUT,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))

            def stop():
                if simulator.poll() is None:
                    simulator.terminate()
                simulator.wait(10)
            resources.callback(stop)
            camera = output / "simulation/camera_session.json"
            deadline = time.monotonic() + 15
            while not camera.exists():
                if simulator.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Simulator did not publish the camera session; inspect simulator.log")
                time.sleep(.05)
            vision_output = output / "vision"
            vision_output.mkdir()
            args = SimpleNamespace(output=vision_output, camera_session=camera, work_dir=None, profile="infantry",
                library=None, dll_dir=[], debug_session=output / "debug-session.json", parameters=None,
                debug=False, debug_hidden=False, record=vision_output / "result.mp4", video_fps=20., duration=0., frames=0)
            session = Session(args, resources)

            def pump(count=10):
                target, deadline = session.frames + count, time.monotonic() + 10
                while session.frames < target:
                    if time.monotonic() > deadline:
                        raise AssertionError("Vision did not continue processing")
                    args.duration = time.perf_counter() - session.began + .1
                    session.run()

            def issue(operation, **values):
                identifier = str(time.monotonic_ns())
                reply = command(args.debug_session, operation, request_id=identifier, **values)
                require(reply == {"requested": True}, f"{operation} wire request accepted")
                pump(2)
                events = [json.loads(line) for line in (vision_output / "debug_events.jsonl").read_text().splitlines()]
                return next(event for event in events if event.get("request_id") == identifier)

            pump(30)
            require(session.result["state"] == "TRACKING" and session.result["aim"]["valid"], "real RGB reaches C++ tracking and valid aim")
            status = command(args.debug_session, "inspect")["status"]
            require(status["context"] == session.pipeline.handle and status["frames"] > 0, "debug inspection uses the live C++ context")
            revision = session.pipeline.revision
            applied = issue("configure", parameters={"camera": {"analog_gain": 200}, "detector": {"binary_val": 121}, "tracker": {"text_size": 1.5}})
            require(not applied["error"] and session.pipeline.revision == revision + 1, "camera/detector/tracker apply as one parameter revision")
            require(session.packet["sensor"]["parameters"]["analog_gain"] == 200 and session.parameters()["detector"]["binary_val"] == 121,
                    "processed frame records the applied camera response and detector parameters")
            for changes in ({"camera": {"flip_image": False}}, {"detector": {"roi_scale": 0}}, {"model": "missing-model.onnx"}):
                before = session.pipeline.handle, session.pipeline.revision, session.parameters()
                failure = issue("configure", parameters=changes)
                require(bool(failure["error"]) and before == (session.pipeline.handle, session.pipeline.revision, session.parameters()),
                        f"invalid parameters preserve the live context: {changes}")
            saved = output / "parameters.json"
            require(not issue("save_parameters", path=str(saved))["error"], "save applied parameters through debug transport")
            require(not issue("configure", parameters={"detector": {"binary_val": 120}})["error"], "second parameter change applies")
            require(not issue("load_parameters", path=str(saved))["error"] and session.parameters() == read(saved), "load restores the saved parameters")
            context, frames = session.pipeline.handle, session.frames
            require(not issue("show")["error"] and session.ui.window.winfo_viewable(), "visible Tk debug window opens")
            require(session.ui.entries["detector"]["binary_val"].get() in ("121", "121.0"), "editor displays applied runtime values")
            revision = session.pipeline.revision
            current = session.ui.last_frame[1]
            pixels = {name: frame.copy() for name, frame in session.ui.images.items() if frame is not None}
            for theme in ("Night", "Day"):
                session.ui.theme.set(theme)
                session.ui.theme_selector.event_generate("<<ComboboxSelected>>")
                session.ui.pump()
                snapshot = session.ui.save_frame()
                require(read(snapshot / "display.json")["theme"] == theme, f"actual Tk selector applies {theme} theme")
                shutil.copy2(snapshot / "window.png", output / f"python-{theme.lower()}.png")
                require(read(snapshot / "parameters.json")["camera"] == current["sensor"]["parameters"], "saved parameters belong to the displayed frame")
            session.ui.font_size.set(20)
            session.ui.font_selector.event_generate("<<ComboboxSelected>>")
            session.ui.pump()
            for name, frame in pixels.items():
                np.testing.assert_array_equal(frame, session.ui.images[name])
            require((context, revision) == (session.pipeline.handle, session.pipeline.revision), "theme and UI font preserve tracker context, revision and all image pixels")
            # Advancing the core without updating the view must not mix two frames when saving.
            session.pipeline.debug_image = lambda _: (_ for _ in ()).throw(AssertionError("saved view must use its own image cache"))
            try:
                session.ui.save_frame()
            finally:
                del session.pipeline.debug_image
            require(True, "saving a displayed frame never queries newer core debug images")
            session.ui.font_size.set(18)
            session.ui.apply_font()
            before = session.packet["image"].copy(), session.packet["K"].copy()
            for index, view in enumerate(session.ui.view_options):
                for key, value in {"smooth": True, "zoom": index == 0, "grid": True, "rotate": 90 * index}.items():
                    view[key].set(value)
            session.ui.redraw()
            np.testing.assert_array_equal(before[0], session.packet["image"])
            np.testing.assert_array_equal(before[1], session.packet["K"])
            require(True, "smooth/1:1/grid/rotation redraw preserves RGB and calibration")
            ui_saved = output / "ui-parameters.json"
            with patch("rm_vision_windows.vision_debug.filedialog.asksaveasfilename", return_value=str(ui_saved)):
                session.ui.save_settings()
            session.ui.entries["detector"]["binary_val"].set("122")
            session.ui.apply()
            require(session.parameters()["detector"]["binary_val"] == 122, "actual Tk Apply callback changes the pipeline")
            with patch("rm_vision_windows.vision_debug.filedialog.askopenfilename", return_value=str(ui_saved)):
                session.ui.load_settings()
            require(session.parameters()["detector"]["binary_val"] == 121, "actual Tk Save/Load callbacks restore configuration (dialogs supplied by test)")
            pump(5)
            require(not issue("save")["error"], "save all current debug images and parameters")
            snapshots = sorted(vision_output.glob("g*_p*_f*/display.json"))
            require(bool(snapshots) and read(snapshots[-1])["desktop_capture"], "window.png is a real visible desktop capture")
            checks["screenshot"] = str(snapshots[-1].parent / "window.png")
            context, frames = session.pipeline.handle, session.frames
            require(not issue("close")["error"] and session.ui is None, "debug window closes")
            pump()
            require(not issue("show")["error"] and session.pipeline.handle == context and session.frames > frames,
                    "closing and reopening debug keeps the C++ context and frame processing")
            from tkinter import TclError
            with patch.object(session.ui, "pump", side_effect=TclError("acceptance: window destroyed")):
                pump()
            require(session.ui is None and session.pipeline.handle == context, "a Tk display error closes only the window; tracking continues")
            session.process(session.packet)
            require(session.packet is None and session.result is None, "duplicate timestamp cannot leave a stale packet paired with an empty result")
            pump()
            require(not issue("configure", parameters={"tracker": {"display": False}})["error"], "disable tracker display")
            recorded = session.video_frames
            pump()
            require(session.video_frames == recorded, "display disabled stops recording")
            require(not issue("configure", parameters={"tracker": {"display": True}})["error"], "enable tracker display")
            pump()
            require(session.video_frames > recorded, "recording resumes with display")
            revision = session.pipeline.revision
            require(not issue("reset")["error"] and session.pipeline.revision == revision + 1, "explicit reset rebuilds the tracker revision")
            checks.update(frames=session.frames, video_frames=session.video_frames,
                          parameter_revision=session.pipeline.revision, feedback_requests=session.feedback_sent)
        video = cv2.VideoCapture(str(vision_output / "result.mp4"))
        decoded = 0
        while video.read()[0]:
            decoded += 1
        video.release()
        require(decoded == checks["video_frames"] and decoded > 0, "recorded MP4 decodes to the actual number of written frames")
        checks["passed"] = True
    except Exception as error:
        checks["error"] = f"{type(error).__name__}: {error}"
    (output / "checks.json").write_text(json.dumps(checks, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(checks, indent=2, ensure_ascii=False))
    return int(not checks["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
