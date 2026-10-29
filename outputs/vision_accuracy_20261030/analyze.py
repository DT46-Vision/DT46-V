"""Offline shot and image diagnostics. Truth never enters the online visual pipeline."""
import bisect
import csv
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent


def records(path):
    with path.open(encoding="utf-8-sig") as stream:
        for line in stream:
            yield json.loads(line)


def nearest(times, t):
    i = min(len(times)-1, bisect.bisect_left(times, t))
    return i-1 if i and abs(times[i-1]-t) < abs(times[i]-t) else i


def stats(values):
    a = np.asarray(values)
    return {"count": len(a), "mean": float(np.mean(a)), "std": float(np.std(a)),
            "abs_p95": float(np.percentile(np.abs(a), 95)), "abs_max": float(np.max(np.abs(a)))} if len(a) else {}


def trace_to_planes(positions, velocities, centers, normals):
    """Independent free-flight RK4; stop at each static armor front plane."""
    p, v = positions.copy(), velocities.copy()
    crossing = np.full_like(p, np.nan)
    flight = np.full(len(p), np.nan)
    dt = .0005
    def acceleration(x):
        return np.array([0., 0., -9.81]) - .019 * np.linalg.norm(x, axis=1)[:, None] * x
    for step in range(1600):
        a = acceleration(v); vb = v + a*dt/2; b = acceleration(vb)
        vc = v + b*dt/2; c = acceleration(vc); vd = v + c*dt; d = acceleration(vd)
        nxt = p + dt/6*(v+2*vb+2*vc+vd)
        nv = v + dt/6*(a+2*b+2*c+d)
        before = np.sum((p-centers)*normals, axis=1)
        after = np.sum((nxt-centers)*normals, axis=1)
        passed = (before >= 0) & (after <= 0) & np.isnan(flight)
        fraction = before[passed]/(before[passed]-after[passed])
        crossing[passed] = p[passed] + fraction[:, None]*(nxt[passed]-p[passed])
        flight[passed] = (step+fraction)*dt
        p, v = nxt, nv
        if np.isfinite(flight).all():
            break
    return crossing, flight


def analyze(run):
    run = run.resolve()
    checks = json.loads((run/"checks.json").read_text(encoding="utf-8-sig"))
    plan = json.loads((run/"scenario.json").read_text(encoding="utf-8-sig"))
    end = plan["duration_s"]
    frames = list(records(run/"vision/frames.jsonl"))
    settings = next(records(run/"vision/parameter_revisions.jsonl"))["settings"]["tracker"]
    truth = list(records(run/"session/cameras/0/truth.jsonl"))
    tt = [r["stamp"]["time_s"] for r in truth]
    events = [r.get("event", {}) for r in records(run/"session/events.jsonl")]
    shots = {e["shot_id"]: e for e in events if e.get("type") == "shot"}
    exits = {e["shot_id"]: e for e in events if e.get("type") == "muzzle_exit"}
    contacts = {}
    for e in events:
        if e.get("type") == "hit" and e.get("geom") == "robot_target3":
            contacts.setdefault(e["shot_id"], e)
    accepted = {e["shot_id"] for e in events if e.get("type") == "armor_judgement" and e.get("accepted") and e.get("target") == "target3"}
    result = {"run": str(run.relative_to(ROOT)), "seed": checks["seed"], "spread_deg": checks["angle_spread_deg"],
              "translation": checks["translation"], "rotation": plan["vehicles"]["target3"]["decision"]["rotation_mode"],
              "shots": len(shots), "damaging": len(accepted), "damaging_per_second": len(accepted)/end,
              "q_xyz": settings["ekf_QR_q_xyz"], "r_xyz_factor": settings["ekf_QR_r_xyz_factor"],
              "invalid_permits": checks["invalid_fire_frames"], "blockers": checks["blockers"]}
    # Motion cases need a moving plane; do not apply this static flight reconstruction to them.
    static = result["translation"] == "stationary" and result["rotation"] == "off"
    if not static:
        mature = {sid for sid, shot in shots.items() if shot["time_s"] < end-.35}
        result.update(mature_shots=len(mature), mature_damaging=len(accepted & mature), maturity_method="launch before end minus 0.35 s")
        return result
    poses = []
    for row in records(run/"session/states.jsonl"):
        state = row["state"]
        poses.append((state["stamp"]["time_s"], state["robots"]["target3"]["chassis"]["T_world_from_chassis"]))
    pt = [p[0] for p in poses]
    ids, positions, velocities, centers, normals, horizontal, vertical = [], [], [], [], [], [], []
    side_angles = {"front": 0, "left": np.pi/2, "rear": np.pi, "right": -np.pi/2}
    for sid, e in exits.items():
        t = e["time_s"]; tr = truth[nearest(tt, t+.2)]
        rotation = np.asarray(poses[nearest(pt, t+.2)][1])[:3, :3]
        best = None
        for armor in tr["armors"]:
            angle = side_angles[armor["side"]]
            n = rotation @ np.array([np.cos(angle)*np.cos(np.pi/12), np.sin(angle)*np.cos(np.pi/12), np.sin(np.pi/12)])
            # Truth is the light-plane site at local z=1.8 mm; CAD front face is z=1.6 mm.
            center = np.asarray(armor["position_world_m"]) - .0002*n
            view = np.asarray(e["position_world_m"])-center
            score = np.dot(n, view)/np.linalg.norm(view)
            if best is None or score > best[0]:
                best = (score, center, n)
        _, center, n = best
        h = np.cross([0, 0, 1], n); h /= np.linalg.norm(h)
        ids.append(sid); positions.append(e["position_world_m"]); velocities.append(e["velocity_world_m_s"])
        centers.append(center); normals.append(n); horizontal.append(h); vertical.append(np.cross(n, h))
    centers, normals, horizontal, vertical = map(np.asarray, (centers, normals, horizontal, vertical))
    crossing, flight = trace_to_planes(np.asarray(positions), np.asarray(velocities), centers, normals)
    offsets = np.column_stack((np.sum((crossing-centers)*horizontal, axis=1), np.sum((crossing-centers)*vertical, axis=1)))
    rows = []
    for i, sid in enumerate(ids):
        arrival = exits[sid]["time_s"]+flight[i]
        censored = sid not in accepted and arrival > end
        inside = abs(offsets[i, 0]) <= .0675+.0085 and abs(offsets[i, 1]) <= .0625+.0085
        proxy_offset = np.full(2, np.nan)
        if sid in contacts:
            contact = contacts[sid]
            point = np.array(contact["position_world_m"])
            velocity = np.array(contact["velocity_before_m_s"])
            point += np.dot(centers[i]-point, normals[i])/np.dot(velocity, normals[i])*velocity
            proxy_offset = np.array([np.dot(point-centers[i], horizontal[i]), np.dot(point-centers[i], vertical[i])])
        proxy_outside = np.isfinite(proxy_offset).all() and (abs(proxy_offset[0])>.076 or abs(proxy_offset[1])>.071)
        rows.append({"shot_id": sid, "launch_s": shots[sid]["time_s"], "arrival_s": arrival,
                     "lateral_m": offsets[i, 0], "vertical_m": offsets[i, 1], "damaging": sid in accepted,
                     "proxy_projection_lateral_m": proxy_offset[0], "proxy_projection_vertical_m": proxy_offset[1],
                     "censored": bool(censored), "free_flight_inside_expanded_rectangle": bool(inside),
                     "classification": "damaging" if sid in accepted else "still_in_flight" if censored else "outside_panel" if not inside else "outside_proxy_panel" if proxy_outside else "requires_review"})
    with (run/"shot-errors.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)
    mature = [r for r in rows if not r["censored"]]
    result.update(mature_shots=len(mature), mature_damaging=sum(r["damaging"] for r in mature),
                  censored=sum(r["censored"] for r in rows), landing_lateral_m=stats([r["lateral_m"] for r in mature]),
                  landing_vertical_m=stats([r["vertical_m"] for r in mature]),
                  requires_review=[r["shot_id"] for r in mature if r["classification"] == "requires_review"])
    errors = {key: [] for key in ("observation_depth_m", "observation_lateral_m", "observation_vertical_m",
                                  "aim_depth_m", "aim_lateral_m", "aim_vertical_m")}
    for f in frames:
        if not f["can_fire"]:
            continue
        tr = truth[nearest(tt, f["simulation_time_s"])]; optical = np.asarray(f["input"]["optical_transform"])
        rotation, origin = optical[:3, :3], optical[:3, 3]
        aim = origin+rotation@np.asarray(f["tracker_debug"]["target_camera_m"])
        # Same static board, matched by image bearing; do not hide depth bias by selecting a side board.
        body_rotation = np.asarray(poses[nearest(pt, f["simulation_time_s"])][1])[:3, :3]
        visible = []
        for armor in tr["armors"]:
            angle = side_angles[armor["side"]]
            normal = body_rotation @ np.array([np.cos(angle)*np.cos(np.pi/12), np.sin(angle)*np.cos(np.pi/12), np.sin(np.pi/12)])
            point = np.asarray(armor["position_world_m"])
            if np.dot(normal, origin-point) > 0:
                visible.append(point)
        board = min(visible,
                    key=lambda p: np.linalg.norm((p-origin)/np.linalg.norm(p-origin)-(aim-origin)/np.linalg.norm(aim-origin)))
        los = (board-origin)/np.linalg.norm(board-origin); h = np.cross([0, 0, 1], los); h /= np.linalg.norm(h)
        for kind, points in (("aim", [aim]), ("observation", [origin+rotation@np.asarray(a["position_camera_m"]) for a in f["observations"] if a["armor_id"] == f["tracked_id"]])):
            for point in points:
                error = point-board
                for axis, basis in (("depth", los), ("lateral", h), ("vertical", np.array([0, 0, 1]))):
                    errors[kind+"_"+axis+"_m"].append(float(np.dot(error, basis)))
    result.update({key: stats(values) for key, values in errors.items()})
    return result


if __name__ == "__main__":
    summaries = []
    for path in sorted(ROOT.glob("*/*/checks.json")):
        summary = analyze(path.parent); summaries.append(summary)
        print(summary["run"], "mature hits/shots", summary["mature_damaging"], summary["mature_shots"],
              "review", summary.get("requires_review", []), flush=True)
    (ROOT/"summary.json").write_text(json.dumps(summaries, indent=2), encoding="utf-8")
