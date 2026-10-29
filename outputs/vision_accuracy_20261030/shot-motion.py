"""Split translation misses into issued-aim prediction and executed projectile error."""
import argparse
import bisect
import csv
import json
from pathlib import Path

import numpy as np
from analyze import records, trace_to_planes, stats


def analyze(run):
    duration = json.loads((run/"scenario.json").read_text(encoding="utf-8-sig"))["duration_s"]
    truth = list(records(run/"session/cameras/0/truth.jsonl"))
    times = np.array([r["stamp"]["time_s"] for r in truth])
    boards = np.array([next(a["position_world_m"] for a in r["armors"] if a["side"] == "right") for r in truth])
    velocity = np.array([r["velocity_world_m_s"][0] for r in truth])
    frames = {r["frame_number"]: r for r in records(run/"vision/frames.jsonl")}
    actions = [r for r in records(run/"session/actions.jsonl") if r.get("kind") == "execution" and r["robot"] == "infantry3"]
    at = [a["stamp"]["time_s"] for a in actions]
    events = [r.get("event", {}) for r in records(run/"session/events.jsonl")]
    shots = {e["shot_id"]: e for e in events if e.get("type") == "shot"}
    exits = [e for e in events if e.get("type") == "muzzle_exit"]
    accepted = {e["shot_id"] for e in events if e.get("type") == "armor_judgement" and e.get("accepted") and e.get("target") == "target3"}
    centers = np.array([[np.interp(e["time_s"]+.2, times, boards[:, k]) for k in range(3)] for e in exits])
    # Right plate on a non-spinning chassis: lateral translation does not change its plane.
    normals = np.tile([0., -np.cos(np.pi/12), np.sin(np.pi/12)], (len(exits), 1))
    centers -= .0002*normals  # 1.8 mm truth site -> 1.6 mm front face
    crossing, flight = trace_to_planes(np.array([e["position_world_m"] for e in exits]),
        np.array([e["velocity_world_m_s"] for e in exits]), centers, normals)
    rows = []
    for i, e in enumerate(exits):
        shot = shots[e["shot_id"]]; t = shot["time_s"]; arrival = e["time_s"]+flight[i]
        if arrival > times[-1] or t >= duration-.35:
            continue
        action = actions[max(0, bisect.bisect_right(at, t)-1)]
        frame = frames.get(action["aim"]["frame_number"])
        if frame is None or "target_camera_m" not in frame["tracker_debug"]:
            continue
        optical = np.array(frame["input"]["optical_transform"])
        aim = optical[:3, 3]+optical[:3, :3]@np.array(frame["tracker_debug"]["target_camera_m"])
        muzzle = np.array(e["position_world_m"])
        true_x = np.interp(arrival, times, boards[:, 0])
        planned_x = muzzle[0]+(aim[0]-muzzle[0])*(crossing[i, 1]-muzzle[1])/(aim[1]-muzzle[1])
        vx = np.interp(t, times, velocity); future_vx = np.interp(arrival, times, velocity)
        rows.append(dict(shot_id=e["shot_id"], launch_s=t, arrival_s=arrival, damaging=e["shot_id"] in accepted,
                         frame_number=frame["frame_number"], frame_age_s=t-frame["simulation_time_s"],
                         cruise=abs(abs(vx)-1.5)<.01 and abs(future_vx-vx)<.01,
                         actual_x_error_m=crossing[i, 0]-true_x, prediction_x_error_m=planned_x-true_x,
                         execution_x_error_m=crossing[i, 0]-planned_x,
                         signed_execution_m=(crossing[i, 0]-planned_x)*np.sign(vx)))
    with (run/"shot-motion.csv").open("w", newline="", encoding="utf-8") as stream:
        writer=csv.DictWriter(stream, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    summary = {}
    for group, selected in (("all", rows), ("cruise", [r for r in rows if r["cruise"]]), ("turning", [r for r in rows if not r["cruise"]])):
        summary[group] = {"shots": len(selected), "damage": sum(r["damaging"] for r in selected),
            **{key: stats([r[key] for r in selected]) for key in ("frame_age_s", "actual_x_error_m", "prediction_x_error_m", "execution_x_error_m", "signed_execution_m")}}
    (run/"shot-motion-summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps({"run": str(run), "groups": {g: {"shots": v["shots"], "damage": v["damage"],
        "age_p95_ms": round(v["frame_age_s"]["abs_p95"]*1000, 1),
        "prediction_p95_mm": round(v["prediction_x_error_m"]["abs_p95"]*1000, 1),
        "execution_p95_mm": round(v["execution_x_error_m"]["abs_p95"]*1000, 1),
        "execution_mean_along_motion_mm": round(v["signed_execution_m"]["mean"]*1000, 1)} for g,v in summary.items()}}))


if __name__ == "__main__":
    parser=argparse.ArgumentParser(); parser.add_argument("run", type=Path)
    analyze(parser.parse_args().run)
