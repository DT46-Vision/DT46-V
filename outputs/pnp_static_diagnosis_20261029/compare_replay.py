"""Compare the recorded user session with a replay using identical selected PnP poses."""
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parent
SIM = OUT.parents[2]
SOURCE = SIM / 'outputs/vscode/aim_hp_review_20261029_185553/cameras/0/vision/frames.jsonl'
AFTER = OUT / 'raw-normal-replay/replayed.jsonl'

def read(path):
    return [json.loads(s) for s in path.open(encoding='utf-8')]

def evaluate(frames):
    result = []
    for f in frames:
        if not f['observations'] or f['state'] != 'TRACKING':
            continue
        o = np.array(f['input']['optical_transform'])[:3, 3]
        x = np.array(f['tracker_debug']['body_ekf_state'])
        center = o[:2] + x[[0, 2]]
        result.append([f['simulation_time_s'], np.linalg.norm(center-[0, -2])*100])
    return np.array(result)

before, after = read(SOURCE), read(AFTER)
assert len(before) == len(after)
for a, b in zip(before, after):
    assert a['simulation_time_s'] == b['simulation_time_s']
    assert a['observations'] == b['observations'], 'Replay changed a selected PnP observation'
old, new = evaluate(before), evaluate(after)
summary = {'frames': len(before), 'identical_observations': True, 'window_s': [74, 88]}
for label, frames, series in [('before', before, old), ('after', after, new)]:
    late = series[(series[:, 0]>=74) & (series[:, 0]<=88), 1]
    f = min(frames, key=lambda f: abs(f['simulation_time_s']-77.72))
    x = np.array(f['tracker_debug']['body_ekf_state'])
    origin = np.array(f['input']['optical_transform'])[:3, 3]
    a = f['observations'][0]
    normal = np.array(f['input']['optical_rotation']) @ a['normal_camera']
    true_normal = np.array([np.cos(np.pi/12), 0, np.sin(np.pi/12)])
    summary[label] = {'late_center_rmse_cm': float(np.sqrt(np.mean(late**2))),
                      'screenshot_time_s': f['simulation_time_s'],
                      'screenshot_center_world_m': (origin + x[[0, 2, 4]]).tolist(),
                      'screenshot_body_yaw_deg': float(np.degrees(x[6])),
                      'screenshot_raw_pnp_normal_error_deg': float(np.degrees(np.arccos(np.clip(normal @ true_normal, -1, 1))))}
(OUT/'replay-comparison.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
print(json.dumps(summary, indent=2))

# Standalone inspection artifact; raw frame logs remain local.
im = Image.new('RGB', (1200, 600), '#101824')
d = ImageDraw.Draw(im)
font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 20)
title = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 27)
d.text((55, 25), 'Same recorded PnP input: stationary target, moving observer', font=title, fill='white')
d.text((55, 64), 'Horizontal body-center error (cm). Removing synthetic yaw reflections only.', font=font, fill='#b5c7dc')
box = (85, 120, 1150, 500)
for value in range(0, 61, 10):
    y = box[3] - value/60*(box[3]-box[1])
    d.line([(box[0], y), (box[2], y)], fill='#2d3a4d')
    d.text((38, y-12), str(value), font=font, fill='#b5c7dc')
for value in range(0, 91, 10):
    x = box[0] + value/90*(box[2]-box[0])
    d.text((x-8, box[3]+10), str(value), font=font, fill='#b5c7dc')
d.text((550, 548), 'Simulation time (s)', font=font, fill='#b5c7dc')
for series, color, label, y in [(old, '#ff8c69', 'Before', 100), (new, '#55d9c0', 'After', 130)]:
    segments = []
    for t, value in series:
        point = (box[0]+t/90*(box[2]-box[0]), box[3]-min(value, 60)/60*(box[3]-box[1]))
        if segments and t-segments[-1][0]>.1:
            if len(segments)>1: d.line([p for _,p in segments], fill=color, width=2)
            segments=[]
        segments.append((t, point))
    if len(segments)>1: d.line([p for _,p in segments], fill=color, width=2)
    d.text((960, y), label, font=font, fill=color)
im.save(OUT/'same-input-comparison.png')
