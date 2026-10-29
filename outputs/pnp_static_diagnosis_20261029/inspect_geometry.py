"""Offline AM02 projection audit. Truth is never supplied to the online pipeline."""
import json
from pathlib import Path
import numpy as np

SIM = Path(__file__).resolve().parents[3]
RUN = SIM / 'outputs/vscode/aim_hp_review_20261029_185553/cameras/0'

def project(points, sensor):
    rotation = np.array(sensor['optical_rotation'])
    origin = np.array(sensor['optical_transform'])[:3, 3]
    camera = (np.asarray(points) - origin) @ rotation
    xy = camera[:, :2] / camera[:, 2:]
    x, y = xy.T
    k1, k2, p1, p2, k3 = sensor['D']
    r2 = x*x + y*y
    scale = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2
    distorted = np.array([x*scale + 2*p1*x*y + p2*(r2+2*x*x),
                          y*scale + p1*(r2+2*y*y) + 2*p2*x*y]).T
    k = np.array(sensor['K'])
    return distorted @ k[:2, :2].T + k[:2, 2]

def hull(points):
    points = sorted(set(map(tuple, points)))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for group, sequence in ((lower, points), (upper, reversed(points))):
        for p in sequence:
            while len(group) >= 2 and cross(group[-2], group[-1], p) <= 0:
                group.pop()
            group.append(p)
    return np.array(lower[:-1] + upper[:-1])

def rectangle_endpoints(points):
    polygon = hull(points)
    best = None
    for edge in np.roll(polygon, -1, axis=0)-polygon:
        u = edge / np.linalg.norm(edge)
        v = np.array([-u[1], u[0]])
        axes = np.array([u, v]).T
        local = polygon @ axes
        low, high = local.min(axis=0), local.max(axis=0)
        extent = high-low
        area = np.prod(extent)
        if best is None or area < best[0]:
            along = np.argmax(extent)
            ends = np.tile((low+high)/2, (2, 1))
            ends[:, along] = [low[along], high[along]]
            best = area, ends @ axes.T
    ends = best[1]
    return ends[np.argsort(-ends[:, 1])]

mesh = []
for side in ('l', 'r'):
    vertices = [list(map(float, line.split()[1:4])) for line in
                (SIM / f'assets/robots/model_levels/medium/am02_light_{side}_cap.obj').read_text(encoding='utf-8').splitlines()
                if line.startswith('v ')]
    mesh.append(np.array(vertices))
print('CAD light bounds:', [(v.min(axis=0).tolist(), v.max(axis=0).tolist()) for v in mesh])
truth = [json.loads(line) for line in (RUN/'truth.jsonl').open(encoding='utf-8')]
truth_by_generation = {}
for tr in truth:
    truth_by_generation.setdefault(tr['stamp']['generation'], []).append(tr)
rows = []
for line in (RUN/'vision/frames.jsonl').open(encoding='utf-8'):
    frame = json.loads(line)
    if not frame['observations'] or frame['frame_number'] % 40:
        continue
    choices = truth_by_generation[frame['reset_generation']]
    times = np.array([tr['stamp']['time_s'] for tr in choices])
    tr = choices[int(np.argmin(abs(times-frame['simulation_time_s'])))]
    if abs(tr['stamp']['time_s']-frame['simulation_time_s']) > .02:
        continue
    if np.linalg.norm(tr['velocity_world_m_s']) > 1e-6 or abs(tr['yaw_rate_rad_s']) > 1e-6:
        continue
    sensor = frame['input']
    rotation = np.array(sensor['optical_rotation'])
    origin = np.array(sensor['optical_transform'])[:3, 3]
    for observation in frame['observations']:
        measured = origin + rotation @ observation['position_camera_m']
        board = min(tr['armors'], key=lambda b: np.linalg.norm(np.array(b['position_world_m'])-measured))
        centre = np.array(board['position_world_m'])
        if np.linalg.norm(centre-measured) > .2:
            continue
        outward = centre - np.array(tr['centre_world_m'])
        outward[2] = 0
        outward /= np.linalg.norm(outward)
        xaxis = np.cross([0, 0, 1], outward)
        yaxis = -np.sin(np.pi/12)*outward + np.array([0, 0, np.cos(np.pi/12)])
        normal = np.cos(np.pi/12)*outward + np.array([0, 0, np.sin(np.pi/12)])
        basis = np.array([xaxis, yaxis, normal]).T
        observed = np.array(observation['corners_px'])
        # Geometry is placed 0.2 mm behind the separately reported sticker-surface centre.
        projected_mesh = [project(v @ basis.T + centre - .0002*normal, sensor) for v in mesh]
        endpoints = np.concatenate([rectangle_endpoints(p) for p in projected_mesh])
        if endpoints[0, 0] > endpoints[2, 0]:
            endpoints = endpoints[[2, 3, 0, 1]]
        nominal = np.array([[-.0675, -.029, 0], [-.0675, .029, 0], [.0675, -.029, 0], [.0675, .029, 0]])
        nominal_px = project(nominal @ basis.T + centre, sensor)
        if nominal_px[0, 0] > nominal_px[2, 0]:
            nominal_px = nominal_px[[2, 3, 0, 1]]
        measured_normal = rotation @ observation['normal_camera']
        normal_error = np.degrees(np.arccos(np.clip(normal @ measured_normal, -1, 1)))
        row = dict(time=frame['simulation_time_s'], side=board['side'],
                   distance_m=float(np.linalg.norm(centre-origin)),
                   position_error_mm=float(np.linalg.norm(centre-measured)*1000),
                   normal_error_deg=float(normal_error),
                   nominal_endpoint_rmse_px=float(np.sqrt(np.mean(np.sum((nominal_px-observed)**2, axis=1)))),
                   mesh_endpoint_rmse_px=float(np.sqrt(np.mean(np.sum((endpoints-observed)**2, axis=1)))),
                   observed=observed.tolist(), mesh_endpoints=endpoints.tolist(), nominal=nominal_px.tolist())
        rows.append(row)
Path(__file__).with_name('geometry-audit.json').write_text(json.dumps(rows, indent=2))
for row in rows[::max(1, len(rows)//12)]:
    print({k:v for k,v in row.items() if k not in ('observed', 'mesh_endpoints', 'nominal')})
print('count', len(rows))
