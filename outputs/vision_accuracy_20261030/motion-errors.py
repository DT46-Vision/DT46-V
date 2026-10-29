"""Fixed-input diagnostics for translation; truth is offline scoring only."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from analyze import records, stats


def evaluate(run, frame_path, output):
    truth = list(records(run / "session/cameras/0/truth.jsonl"))
    times = np.array([r["stamp"]["time_s"] for r in truth])
    boards = np.array([next(a["position_world_m"] for a in r["armors"] if a["side"] == "right") for r in truth])
    velocities = np.array([r["velocity_world_m_s"] for r in truth])
    frames = list(records(frame_path))
    source = {r["frame_number"]: r for r in records(run / "vision/frames.jsonl")}
    camera_times = np.array([r["simulation_time_s"] for r in frames])
    camera_x = np.array([r["input"]["optical_transform"][0][3] for r in frames])
    camera_vx = np.gradient(camera_x, camera_times)
    rows = []
    horizon = .23  # approximately 0.20 s flight + configured 0.03 s delays at 5 m
    for index, f in enumerate(frames):
        d = f["tracker_debug"]
        if f["state"] != "TRACKING" or d["observed_plate"] < 0:
            continue
        t = f["simulation_time_s"]
        if t + horizon > times[-1]:
            continue
        body = np.array(d["body_ekf_state"])
        optical = np.array(f["input"]["optical_transform"])
        rotation, origin = optical[:3, :3], optical[:3, 3]
        true_x = np.interp(t, times, boards[:, 0])
        future_x = np.interp(t+horizon, times, boards[:, 0])
        true_vx = np.interp(t, times, velocities[:, 0])
        future_vx = np.interp(t+horizon, times, velocities[:, 0])
        plate = d["observed_plate"]
        theta = body[6] + plate*np.pi/2
        radius = body[8+plate % 2]
        estimate_x = origin[0]+body[0]-radius*np.cos(theta)
        forecast_x = origin[0]+body[0]+body[1]*horizon-radius*np.cos(theta+body[7]*horizon)
        points = [origin+rotation@np.array(a["position_camera_m"]) for a in f["observations"] if a["armor_id"] == f["tracked_id"]]
        if not points:
            continue
        # The right board is the facing board in this non-spinning translation scenario.
        true_board = np.array([np.interp(t, times, boards[:, k]) for k in range(3)])
        future_board = np.array([np.interp(t+horizon, times, boards[:, k]) for k in range(3)])
        los = future_board-origin
        lateral = np.array([los[1], -los[0], 0.]); lateral /= np.linalg.norm(lateral)
        current_los = true_board-origin
        current_lateral = np.array([current_los[1], -current_los[0], 0.]); current_lateral /= np.linalg.norm(current_lateral)
        forecast = origin+np.array([body[0]+body[1]*horizon-radius*np.cos(theta+body[7]*horizon),
                                   body[2]+body[3]*horizon-radius*np.sin(theta+body[7]*horizon),
                                   body[4]+body[5]*horizon+(plate % 2)*body[10]])
        point = min(points, key=lambda p: np.linalg.norm((p-origin)/np.linalg.norm(p-origin)-(true_board-origin)/np.linalg.norm(true_board-origin)))
        row = dict(time_s=t, frame=f["frame_number"], fire=f["can_fire"], original_fire=source[f["frame_number"]]["can_fire"],
                   cruise=abs(abs(true_vx)-1.5)<.01 and abs(future_vx-true_vx)<.01,
                   true_x=true_x, true_vx=true_vx, estimate_x=estimate_x, estimate_vx=body[1], camera_vx=camera_vx[index],
                   observation_x_error_m=point[0]-true_x, position_x_error_m=estimate_x-true_x,
                   velocity_x_error_m_s=body[1]+camera_vx[index]-true_vx,
                   forecast_x_error_m=forecast_x-future_x,
                   perfect_cv_error_m=true_x+true_vx*horizon-future_x,
                   observation_lateral_error_m=float((point-true_board)@current_lateral),
                   forecast_lateral_error_m=float((forecast-future_board)@lateral))
        rows.append(row)
    output.mkdir(parents=True, exist_ok=True)
    with (output/"motion-errors.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    result = {"horizon_s": horizon, "source": str(frame_path), "groups": {}}
    for group, selected in (("tracking", rows), ("cruise", [r for r in rows if r["cruise"]]),
                            ("turning", [r for r in rows if not r["cruise"]]),
                            ("original_fire", [r for r in rows if r["original_fire"]]),
                            ("fire", [r for r in rows if r["fire"]])):
        result["groups"][group] = {key: stats([r[key] for r in selected]) for key in
            ("observation_x_error_m", "position_x_error_m", "velocity_x_error_m_s", "forecast_x_error_m", "perfect_cv_error_m", "observation_lateral_error_m", "forecast_lateral_error_m")}
        result["groups"][group]["forecast_within_76mm"] = int(sum(abs(r["forecast_lateral_error_m"])<=.076 for r in selected))
    (output/"motion-summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(output), "groups": {k: {"count": v["forecast_x_error_m"]["count"],
        "forecast_p95_mm": round(1000*v["forecast_x_error_m"]["abs_p95"], 1),
        "velocity_error_p95": round(v["velocity_x_error_m_s"]["abs_p95"], 3),
        "raw_position_p95_mm": round(1000*v["observation_x_error_m"]["abs_p95"], 1),
        "raw_lateral_p95_mm": round(1000*v["observation_lateral_error_m"]["abs_p95"], 1),
        "forecast_lateral_p95_mm": round(1000*v["forecast_lateral_error_m"]["abs_p95"], 1),
        "perfect_cv_p95_mm": round(1000*v["perfect_cv_error_m"]["abs_p95"], 1),
        "within_76mm": v["forecast_within_76mm"]} for k, v in result["groups"].items()}}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    parser.add_argument("--frames", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    evaluate(args.run, args.frames or args.run/"vision/frames.jsonl", args.output or args.run)
