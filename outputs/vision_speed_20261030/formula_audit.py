"""Candidate closed-form drag approximation against independent spatial RK4."""
import json
from pathlib import Path
import numpy as np


def candidate(r, z, v, k=.019):
    drag = k * np.hypot(1, z / r)
    q = drag * r
    a = 9.81 / (v * v) * (np.expm1(2 * q) - 2 * q) / (4 * drag * drag)
    slope = 2 * (a + z) / (r + np.sqrt(r * r - 4 * a * (a + z)))
    tof = np.expm1(q) / (drag * v) * np.hypot(1, slope)
    return slope, tof


def reference(r, slope, v, steps=2400, k=.019):
    # y=(height, horizontal speed, vertical/horizontal slope, time), x independent.
    y = np.array([np.zeros_like(r), v / np.hypot(1, slope), slope, np.zeros_like(r)])
    dx = r / steps
    def f(s):
        return np.array([s[2], -k * np.hypot(1, s[2]) * s[1], -9.81 / s[1]**2, 1 / s[1]])
    for _ in range(steps):
        a = f(y); b = f(y + a * dx / 2); c = f(y + b * dx / 2); d = f(y + c * dx)
        y += dx / 6 * (a + 2 * b + 2 * c + d)
    return y[0], y[3]


if __name__ == "__main__":
    r, z, v = (a.ravel() for a in np.meshgrid(np.linspace(.5, 12, 47), np.linspace(-3.5, 3.5, 29), np.linspace(20, 30, 11)))
    slope, tof = candidate(r, z, v)
    height, actual_tof = reference(r, slope, v)
    e = np.abs(height-z); i = np.argmax(e)
    result = {"samples": len(r), "max_height_error_m": float(e[i]), "p95_height_error_m": float(np.percentile(e, 95)),
              "max_time_error_s": float(np.max(np.abs(tof-actual_tof))),
              "worst": {"range_m": r[i], "height_m": z[i], "speed_m_s": v[i]}}
    print(json.dumps(result, indent=2))
    Path(__file__).with_suffix(".json").write_text(json.dumps(result, indent=2))
