"""CPU parity and full-frame evaluation; native requests contain no target truth."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

import numpy as np
import onnxruntime as ort
from PIL import Image


def read_lines(path):
    return [json.loads(line) for line in path.open(encoding="utf-8")]


def rotation(axes):
    x, y = np.array(axes[:3], dtype=float), np.array(axes[3:], dtype=float)
    x /= np.linalg.norm(x)
    y -= x * np.dot(x, y)
    y /= np.linalg.norm(y)
    return np.column_stack((x, y, np.cross(x, y)))


def softmax(logits):
    e = np.exp(logits - np.max(logits))
    return e / e.sum()


def iou(a, b):
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    area = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1])
    return intersection / max(area - intersection, 1)


def metrics(errors, rotations):
    if not errors:
        return {"count": 0}
    e = np.array(errors)
    return {"count": len(e), "depth_bias_m": float(e[:, 2].mean()),
            "depth_absolute_p50_m": float(np.quantile(abs(e[:, 2]), .5)),
            "depth_absolute_p95_m": float(np.quantile(abs(e[:, 2]), .95)),
            "translation_rmse_m": float(np.sqrt(np.square(e).sum(1).mean())),
            "rotation_mean_deg": float(np.mean(rotations))}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--split", choices=["validation", "test"], default="validation")
    parser.add_argument("--opencv-bin", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root, dataset = args.root.resolve(), args.dataset.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    config = json.loads((root / "standalone/armor_model.json").read_text())
    model_path = root / config["model"]
    metadata = json.loads(model_path.with_suffix(".json").read_text())
    env = dict(os.environ)
    env["PATH"] = str(args.opencv_bin.resolve()) + os.pathsep + env["PATH"]
    executable = root / "build/native/bin/dt46_model_eval.exe"

    def process(mode):
        return subprocess.Popen([str(executable), str(root), mode], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)

    def call(child, request):
        child.stdin.write(json.dumps(request) + "\n")
        child.stdin.flush()
        line = child.stdout.readline()
        if not line:
            raise RuntimeError(child.stderr.read())
        return json.loads(line)

    def close(child):
        child.stdin.close()
        child.wait(timeout=30)
        if child.returncode:
            raise RuntimeError(child.stderr.read())

    rows = [r for r in read_lines(dataset / "training/training.jsonl") if r["split"] == args.split]
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    session = ort.InferenceSession(str(model_path), sess_options=options, providers=["CPUExecutionProvider"])
    child = process("crops")
    maxima = {key: 0.0 for key in ["position_m", "rotation_matrix", "presence_score", "sticker_score", "team_score"]}
    crop_counts = {key: 0 for key in ["tp", "fp", "tn", "fn", "sticker_correct", "team_correct", "sticker_supervised", "team_supervised", "positive"]}
    crop_errors, crop_rotations = [], []
    with (args.output / "crops.jsonl").open("w", encoding="utf-8") as out:
        for row in rows:
            path = (dataset / "training" / row["image"]).resolve()
            actual = call(child, {"image": str(path), "K": row["K"], "D": row["D"]})
            pixels = np.array(Image.open(path).convert("RGB"), dtype=np.float32).transpose(2, 0, 1)[None] / 255
            k = np.array(row["K"], dtype=np.float32)
            camera = np.array([np.log(k[0, 0] / 128), np.log(k[1, 1] / 128), k[0, 2] / 128, k[1, 2] / 128, *row["D"]], dtype=np.float32)[None]
            armor, sticker, team, position, axes = session.run(None, {"image": pixels, "camera": camera})
            expected = {"position_m": position[0], "rotation_matrix": rotation(axes[0]),
                        "presence_score": 1 / (1 + np.exp(-float(armor[0]))),
                        "sticker_score": float(softmax(sticker[0]).max()), "team_score": float(softmax(team[0]).max())}
            for key, value in expected.items():
                field = {"position_m": "position_camera_m", "rotation_matrix": "rotation_camera_from_board"}.get(key, key)
                np.testing.assert_allclose(actual[field], value, rtol=1e-4, atol=1e-4, err_msg=key)
                maxima[key] = max(maxima[key], float(np.max(np.abs(np.array(actual[field]) - value))))
            assert actual["sticker_type"] == metadata["classes"][int(sticker[0].argmax())]
            assert actual["team"] == metadata["teams"][int(team[0].argmax())]
            number = {"engineer_2": 2, "infantry_3": 3, "infantry_4": 4, "sentry": 7}.get(actual["sticker_type"], -1)
            expected_id = (-1 if number < 0 else (5 if number == 7 else number - 1) + (6 if actual["team"] == "red" else 0))
            assert actual["robot_number"] == number and actual["armor_id"] == expected_id
            pred, truth = actual["presence_score"] >= .5, row["is_armor"]
            crop_counts[("tp" if truth else "fp") if pred else ("fn" if truth else "tn")] += 1
            if truth:
                target = row["target"]
                crop_counts["positive"] += 1
                for name, field in (("sticker", "sticker_type"), ("team", "team")):
                    if row["supervision"][name]:
                        crop_counts[name + "_supervised"] += 1
                        crop_counts[name + "_correct"] += actual[field] == target[field]
                crop_errors.append(np.array(actual["position_camera_m"]) - target["pose"]["position_camera_m"])
                cosine = (np.sum(np.array(actual["rotation_camera_from_board"]) * target["pose"]["rotation_camera_from_board"]) - 1) / 2
                crop_rotations.append(float(np.degrees(np.arccos(np.clip(cosine, -1, 1)))))
            out.write(json.dumps({"image": row["image"], **actual}) + "\n")
    close(child)
    labels = {r["sample_id"]: r for r in read_lines(dataset / "labels.jsonl")}
    eligible = {}
    for r in rows:
        if r["is_armor"]:
            eligible.setdefault(r["sample_id"], []).append(r["target"])
    frames = [r for r in read_lines(dataset / "frames.jsonl") if r["split"] == args.split]
    counts = {key: 0 for key in ["frames", "positive_targets", "detections", "matched", "fp", "fn", "sticker_correct", "team_correct", "joint_correct", "sticker_supervised", "team_supervised", "joint_supervised", "negative_frames", "negative_frames_with_detections"]}
    errors, rotations, times, candidates = [], [], [], []
    child = process("frames")
    examples = []
    with (args.output / "frames.jsonl").open("w", encoding="utf-8") as out:
        for index, frame in enumerate(frames):
            request = {"image": str(dataset / frame["image"]), "K": frame["K"], "D": frame["D"]}
            if index < 6:
                request["debug_image"] = str((args.output / f"frame_{index}.png").resolve())
            actual = call(child, request)
            observations = actual["observations"]
            targets = eligible.get(frame["sample_id"], [])
            counts["frames"] += 1
            counts["positive_targets"] += len(targets)
            counts["detections"] += len(observations)
            if not labels[frame["sample_id"]]["objects"]:
                counts["negative_frames"] += 1
                counts["negative_frames_with_detections"] += bool(observations)
            pairs = []
            for i, observation in enumerate(observations):
                x, y, w, h = observation["model"]["roi_xywh"]
                for j, target in enumerate(targets):
                    overlap = iou([x, y, x+w, y+h], target["crop"]["roi_xyxy"])
                    if overlap >= .3:
                        pairs.append((overlap, i, j))
            used_o, used_t = set(), set()
            for overlap, i, j in sorted(pairs, reverse=True):
                if i in used_o or j in used_t:
                    continue
                used_o.add(i)
                used_t.add(j)
                o, target = observations[i], targets[j]
                counts["matched"] += 1
                sticker_correct = o["model"]["sticker_type"] == target["sticker_type"]
                team_correct = o["model"]["team"] == target["team"]
                masks = target["supervision"]
                counts["sticker_supervised"] += masks["sticker"]
                counts["team_supervised"] += masks["team"]
                counts["joint_supervised"] += masks["sticker"] and masks["team"]
                counts["sticker_correct"] += sticker_correct and masks["sticker"]
                counts["team_correct"] += team_correct and masks["team"]
                counts["joint_correct"] += sticker_correct and team_correct and masks["sticker"] and masks["team"]
                errors.append(np.array(o["position_camera_m"]) - target["pose"]["position_camera_m"])
                cosine = (np.sum(np.array(o["model"]["rotation_camera_from_board"]) * target["pose"]["rotation_camera_from_board"]) - 1) / 2
                rotations.append(float(np.degrees(np.arccos(np.clip(cosine, -1, 1)))))
            counts["fp"] += len(observations) - len(used_o)
            counts["fn"] += len(targets) - len(used_t)
            times.append([actual["process_ms"], *actual["stage_ms"]])
            candidates.append(actual["candidate_count"])
            if len(examples) < 8 and observations and any(o["model"]["tracker_eligible"] for o in observations):
                examples.append({"request": request, "sample_id": frame["sample_id"], "observations": observations})
            out.write(json.dumps({"sample_id": frame["sample_id"], **actual}) + "\n")
    # A fixed-image sequence checks observation→EKF units, reset and color routing separately from accuracy.
    sequence = []
    if examples:
        request = dict(examples[0]["request"])
        first = call(child, {**request, "reset": True, "time": 0})
        sequence.append(first)
        for tick in range(1, 12):
            sequence.append(call(child, {**request, "reset": False, "time": tick / 60}))
        if not any(r["state"] == "TRACKING" for r in sequence):
            raise AssertionError("Repeated model observations never reached TRACKING")
        chosen = next(o for o in first["observations"] if o["armor_id"] == first["tracked_id"])
        expected = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]) @ chosen["position_camera_m"]
        np.testing.assert_allclose(first["position_camera_origin_world_axes_m"], expected, atol=1e-6, rtol=1e-6)
    close(child)
    timing = np.array(times)
    summary = {"split": args.split, "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
               "native_executable_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
               "configuration": config, "crop_parity": {"samples": len(rows), "max_absolute_error": maxima},
               "crop_counts": crop_counts, "crop_pose": metrics(crop_errors, crop_rotations),
               "full_frame_counts": counts, "matching": "one-to-one padded ROI IoU >= 0.3 against selected truth crops; not object AP",
               "full_frame_pose_matched_only": metrics(errors, rotations),
               "timing_ms": {"columns": ["process", "preparation", "candidates", "model", "tracker", "snapshot"],
                             "p50": np.quantile(timing, .5, axis=0).tolist(), "p95": np.quantile(timing, .95, axis=0).tolist()},
               "candidate_count": {"mean": float(np.mean(candidates)), "p95": float(np.quantile(candidates, .95)), "max": max(candidates)},
               "repeated_frame_states": [r["state"] for r in sequence],
               "limits": "Full-frame errors use only matched detections. Independent frames reset tracker outside timing. Sequence is a wiring check, not dynamic tracking accuracy."}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (args.output / "sequence.json").write_text(json.dumps(sequence, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
