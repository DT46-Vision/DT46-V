"""Rebuild the offline figure and paired replay statistics from saved diagnostics."""
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent.parent/".cache/vision-accuracy-plot"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from analyze import stats


def rows(path):
    with path.open(encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


fig, axes = plt.subplots(2, 2, figsize=(11, 8), constrained_layout=True)
for ax, pattern, title in zip(axes[0], ("normal-seed*/5_off", "noangle-seed*/5_off"),
                            ("5 m static: original angular spread", "5 m static: angular spread disabled")):
    data = [r for folder in ROOT.glob(pattern) for r in rows(folder/"shot-errors.csv") if r["censored"] == "False"]
    for hit, color, label in ((False, "#b34d33", "No armor damage"), (True, "#167c7b", "Armor damage")):
        selected = [r for r in data if (r["damaging"] == "True") == hit]
        ax.scatter([1000*float(r["lateral_m"]) for r in selected], [1000*float(r["vertical_m"]) for r in selected],
                   s=10, alpha=.65, color=color, label=label)
    ax.add_patch(Rectangle((-67.5, -62.5), 135, 125, fill=False, linewidth=1.5, color="#343a40"))
    ax.set(xlim=(-150, 150), ylim=(-150, 150), aspect="equal", xlabel="Lateral offset (mm)", ylabel="Vertical offset (mm)", title=title)
    ax.grid(alpha=.18); ax.legend(loc="lower left", fontsize=8)
old = {int(r["frame"]): r for r in rows(ROOT/"replays/baseline/motion-errors.csv")}
new = {int(r["frame"]): r for r in rows(ROOT/"replays/q03/motion-errors.csv")}
shared = sorted(old.keys() & new.keys())
paired = {}
for group, ids in (("all", shared), ("cruise", [i for i in shared if old[i]["cruise"] == "True"]),
                   ("turning", [i for i in shared if old[i]["cruise"] != "True"] )):
    paired[group] = {name: stats([float(source[i]["forecast_lateral_error_m"]) for i in ids]) for name, source in (("baseline", old), ("accepted", new))}
(ROOT/"paired-replay-summary.json").write_text(json.dumps(paired, indent=2), encoding="utf-8")
ids = [i for i in shared if 3 <= float(old[i]["time_s"]) <= 6]
times = [float(old[i]["time_s"]) for i in ids]
axes[1, 0].plot(times, [float(old[i]["true_vx"])-float(old[i]["camera_vx"]) for i in ids], color="#343a40", linewidth=2, label="True relative velocity")
for source, color, label in ((old, "#b34d33", "Baseline"), (new, "#167c7b", "Adjusted noise")):
    axes[1, 0].plot(times, [float(source[i]["estimate_vx"]) for i in ids], color=color, label=label)
    axes[1, 1].plot(times, [1000*float(source[i]["forecast_lateral_error_m"]) for i in ids], color=color, label=label)
axes[1, 0].set(title="Translation: speed estimate lags reversals", xlabel="Simulation time (s)", ylabel="Relative X velocity (m/s)")
axes[1, 1].axhspan(-76, 76, color="#167c7b", alpha=.1, label="+/-76 mm reference")
axes[1, 1].set(title="Same images: forecast at +0.23 s", xlabel="Simulation time (s)", ylabel="Lateral prediction error (mm)")
for ax in axes[1]:
    ax.grid(alpha=.18); ax.legend(fontsize=8)
fig.suptitle("Separate launcher dispersion from vision prediction error", fontsize=15)
fig.savefig(ROOT/"accuracy-diagnostics.png", dpi=160)
fig.savefig(ROOT/"accuracy-diagnostics.pdf")
print(json.dumps(paired))
