"""Score recorded RGB closed loops; truth is read only after the run ends."""
import json
import math
import statistics
import subprocess
from pathlib import Path

folder = Path(__file__).resolve().parent
root = folder.parent.parent
scorer = root / "build/aim-validation-full/dt46_aim_truth_check.exe"
batches = [
    "baseline-open-20261030-005458", "gate-color-20261030-010740",
    "upright-edge-20261030-011915", "final-motion-20261030-012408",
    "final-motion2-20261030-013028", "final-20261030-013123",
]
results = []
for batch in batches:
    for checks_file in sorted((folder / batch).glob("*/checks.json")):
        run = checks_file.parent
        checks = json.loads(checks_file.read_text(encoding="utf-8-sig"))
        frames = [json.loads(line) for line in (run / "vision/frames.jsonl").open(encoding="utf-8")]
        subprocess.run([str(scorer), str(run / "vision/frames.jsonl"),
                        str(run / "session/cameras/0/truth.jsonl"), str(run / "truth-score")],
                       check=True, stdout=subprocess.DEVNULL)
        truth = json.loads((run / "truth-score/truth-summary.json").read_text())
        accepted = set()
        for line in (run / "session/events.jsonl").open(encoding="utf-8"):
            record = json.loads(line)
            event = record.get("event", {})
            if (record.get("kind") == "referee" and event.get("type") == "armor_judgement"
                    and event.get("accepted") and event.get("target") == "target3"):
                accepted.add(event["shot_id"])
        timing = sorted(frame["processing_ms"] for frame in frames)
        row = {**checks, "batch": batch, "unique_damaging_shots": len(accepted),
               "processing_ms_median": statistics.median(timing),
               "processing_ms_p95": timing[int(.95 * (len(timing)-1))], "truth": truth}
        row.pop("run")
        results.append(row)
        print(batch, checks["case"], "shots/damaging", checks["shots"], len(accepted),
              "active center RMSE", round(truth.get("active_center_error_m", {}).get("rmse", math.nan), 4))
(folder / "comparison.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
