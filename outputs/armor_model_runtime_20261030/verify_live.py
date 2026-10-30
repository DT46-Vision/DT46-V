"""Exercise the real simulator/camera/vision process and debug-window lifecycle."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def wait_for(predicate, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            value = predicate()
            if value:
                return value
        except (FileNotFoundError, json.JSONDecodeError):
            pass
        time.sleep(.1)
    raise RuntimeError("Timed out waiting for simulator/debug evidence")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root, output = args.root.resolve(), args.output.resolve()
    if output.exists():
        raise ValueError("Use a fresh output directory")
    vision = root / "DT46_V/build/native/bin/dt46_vision.exe"
    simulator = root / "build/release/rm_sim.exe"
    command = [str(simulator), "--root", str(root), "--session", "config/sessions/aim_tracking.json",
               "--output", str(output), "--duration", "18", "--hidden"]
    host = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
    try:
        debug = output / "cameras/0/debug.json"
        events_path = output / "cameras/0/vision/debug_events.jsonl"
        wait_for(lambda: read(debug))

        def events():
            return [json.loads(s) for s in events_path.read_text(encoding="utf-8").splitlines() if s]

        for command_name, event in [("--show-debug", "opened"), ("--save-debug", "saved"), ("--close-debug", "closed")]:
            subprocess.run([str(vision), command_name, "--debug-session", str(debug)], check=True, capture_output=True, text=True)
            wait_for(lambda: any(row["event"] == event for row in events()))
        stdout, stderr = host.communicate(timeout=30)
        if host.returncode:
            raise RuntimeError(stdout + stderr)
        report = read(output / "cameras/0/vision/report.json")
        assert report["error"] == ""
        rows = [json.loads(s) for s in (output / "cameras/0/vision/frames.jsonl").read_text(encoding="utf-8").splitlines()]
        states = Counter(r["state"] for r in rows)
        assert states["TRACKING"] > 0
        lifecycle = [e for e in events() if e["event"] in {"opened", "saved", "closed"}]
        assert len({e["context"] for e in lifecycle}) == 1
        provenance = read(output / "cameras/0/provenance.json")
        for entry in provenance["files"]:
            assert hashlib.sha256(Path(entry["path"]).read_bytes()).hexdigest() == entry["sha256"]
        assert any(e["path"].endswith(".onnx") for e in provenance["files"])
        assert not any("detector_params_" in e["path"] for e in provenance["files"])
        summary = {"frames": len(rows), "states": dict(states),
                   "sticker_counts": dict(Counter(o["model"]["sticker_type"] for r in rows for o in r["observations"])),
                   "team_counts": dict(Counter(o["model"]["team"] for r in rows for o in r["observations"])),
                   "can_fire_frames": sum(r["can_fire"] for r in rows),
                   "debug_events": lifecycle, "model_provenance_hashes_verified": True,
                   "simulation_report": str(output / "report.json"), "vision_error": report["error"],
                   "limits": "Stationary target with manual controller; no shooting or moving-target accuracy claim."}
        (output / "model_checks.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(summary, indent=2))
    finally:
        if host.poll() is None:
            host.terminate()
            host.wait(timeout=10)


if __name__ == "__main__":
    main()
