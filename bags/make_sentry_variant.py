#!/usr/bin/env python3
import re
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
q_xyz, r_xyz, q_yaw, q_r = map(float, sys.argv[3:7])
text = src.read_text()
subs = {
    r"ekf_QR_q_xyz:\s*[0-9.]+": f"ekf_QR_q_xyz: {q_xyz}",
    r"ekf_QR_q_yaw:\s*[0-9.]+": f"ekf_QR_q_yaw: {q_yaw}",
    r"ekf_QR_q_r:\s*[0-9.]+": f"ekf_QR_q_r: {q_r}",
    r"ekf_QR_r_xyz_factor:\s*[0-9.]+": f"ekf_QR_r_xyz_factor: {r_xyz}",
}
for pat, repl in subs.items():
    text, n = re.subn(pat, repl, text, count=1)
    if n != 1:
        raise SystemExit(f"failed to replace {pat}")
dst.write_text(text)
print(dst)
