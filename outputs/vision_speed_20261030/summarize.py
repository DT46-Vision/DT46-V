"""Compact evidence for this run; bulky frame logs and timing samples stay local."""
import csv
import hashlib
import json
import math
import re
from pathlib import Path
import statistics

import numpy as np

folder = Path(__file__).resolve().parent
source = folder / "comparison.json"
rows = json.loads(source.read_text())["rows"]
assert len(rows) == 36, "Expected nine frames, two speed modes, two binaries"
summary = {"benchmark_source": source.name, "binaries": json.loads(source.read_text())["dll_sha256"]}
for mode in ("fixed", "changing"):
    summary[mode] = {}
    for variant in ("before", "after"):
        selected = [r for r in rows if r["mode"] == mode and r["variant"] == variant]
        times = [value for r in selected for value in r["samples_ms"]]
        summary[mode][variant] = dict(zip(("median_ms", "p95_ms", "p99_ms", "max_ms"),
                                         np.percentile(times, (50, 95, 99, 100))))
        summary[mode][variant]["startup_median_ms"] = statistics.median(r["startup_ms"] for r in selected)
for before, after in zip(rows[::2], rows[1::2]):
    assert before["observations"] == after["observations"] and before["lights"] == after["lights"]
summary["identical_detection_cases"] = len(rows) // 2
summary["stage_median_across_frames_ms"] = {key: statistics.median(r["stage_median_ms"][key] for r in rows if r["variant"] == "after")
                                          for key in rows[1]["stage_median_ms"]}
summary["frames"] = [{"path": r["frame"], "sha256": hashlib.sha256((folder.parents[1] / r["frame"]).with_name("image_raw.png").read_bytes()).hexdigest()}
                     for r in rows if r["mode"] == "fixed" and r["variant"] == "after"]
grid = list(csv.DictReader((folder / "ballistic_grid.csv").open()))
for label, selected in (("all", grid), ("range_1_to_12_height_abs_0_5", [r for r in grid if float(r["range"]) >= 1 and abs(float(r["height"])) <= .5])):
    summary["ballistics_" + label] = {}
    for key in ("old_error_m", "new_error_m", "old_tof_error_s", "new_tof_error_s"):
        values = [abs(float(r[key])) for r in selected if math.isfinite(float(r[key]))]
        summary["ballistics_" + label][key] = dict(zip(("median", "p95", "max"), np.percentile(values, (50, 95, 100))))
live = []
for batch in sorted(folder.glob("*-20261030-*")):
    for path in sorted(batch.glob("*/checks.json")):
        row = json.loads(path.read_text(encoding="utf-8-sig"))
        row.pop("run")
        row["batch"] = batch.name
        frames = [json.loads(line) for line in (path.parent / "vision/frames.jsonl").open()]
        for key, values in (("processing_ms", [f["processing_ms"] for f in frames]),
                            ("frame_interval_ms", [1000 * (b["simulation_time_s"] - a["simulation_time_s"]) for a, b in zip(frames, frames[1:])])):
            row[key] = dict(zip(("median", "p95", "max"), np.percentile(values, (50, 95, 100))))
        accepted = set()
        for line in (path.parent / "session/events.jsonl").open():
            event = json.loads(line).get("event", {})
            if event.get("type") == "armor_judgement" and event.get("accepted") and event.get("target") == "target3":
                accepted.add(event["shot_id"])
        row["unique_damaging_shots"] = len(accepted)
        live.append(row)
summary["live"] = live
ctest = re.search(r"(\d+)% tests passed, (\d+) tests failed out of (\d+)",
                  (folder / "ctest.log").read_text(encoding="utf-8-sig"))
assert ctest and ctest[1] == "100" and ctest[2] == "0"
summary["validation"] = {"ctest_passed": int(ctest[3]), "ctest_failed": 0,
                         "ballistics": (folder / "ballistics_test.log").read_text(encoding="utf-8-sig").strip()}
(folder / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
for row in live:
    print(row["batch"], row["case"], "shots/damaging", row["shots"], row["unique_damaging_shots"],
          "compute/period", round(row["processing_ms"]["median"], 3), round(row["frame_interval_ms"]["median"], 3))
