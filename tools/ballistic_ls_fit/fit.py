"""静态命中弹道拟合：CSV 七项观测量 + yaml 参数开关 → 非线性最小二乘。

残差：命中姿态下解算 (Δyaw, Δpitch)，期望为 (0, 0)。
算残差时使用全部参数；仅 fit: true 的参数参与优化。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares

DEG2RAD = np.pi / 180.0
RAD2DEG = 180.0 / np.pi
G = 9.81
DT = 0.005

PARAM_KEYS = [
    "k_v2",
    "cam_to_gun_pos_x",
    "cam_to_gun_pos_y",
    "cam_to_gun_pos_z",
    "cam_to_gun_rpy_r",
    "cam_to_gun_rpy_p",
    "cam_to_gun_rpy_y",
]


def _rx(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_to_matrix(rpy_deg: np.ndarray, order: str = "xyz") -> np.ndarray:
    """与 C++ RmTF::euler_to_matrix 一致。

    a0,a1,a2 = 输入向量三个分量（弧度）。
    xyz: Rz(a2)*Ry(a1)*Rx(a0)
    zyx: Rx(a2)*Ry(a1)*Rz(a0)  — a0 挂 Z、a2 挂 X（不是 Rx(a0)*...*Rz(a2)）
    """
    a0, a1, a2 = np.asarray(rpy_deg, dtype=float) * DEG2RAD
    if order == "xyz":
        return _rz(a2) @ _ry(a1) @ _rx(a0)
    if order == "zyx":
        return _rx(a2) @ _ry(a1) @ _rz(a0)
    raise ValueError(order)


def rotate_pos_axis(xyz: np.ndarray, rpy_deg: np.ndarray, order: str = "xyz") -> np.ndarray:
    return euler_to_matrix(rpy_deg, order) @ np.asarray(xyz, dtype=float)


def apply_rotation_rpy(raw_rpy_deg: np.ndarray, rotation_rpy_deg: np.ndarray) -> np.ndarray:
    """对齐 C++ rotate_pose_axis 的姿态叠加，但不做开机 yaw 相对化（逐样本独立）。"""
    r_cur = euler_to_matrix(raw_rpy_deg, "xyz")
    r_fix = euler_to_matrix(rotation_rpy_deg, "xyz")
    r_final = r_cur @ r_fix
    pitch = np.arcsin(np.clip(-r_final[2, 0], -1.0, 1.0))
    yaw = np.arctan2(r_final[1, 0], r_final[0, 0])
    roll = np.arctan2(r_final[2, 1], r_final[2, 2])
    return np.array([roll, pitch, yaw]) * RAD2DEG


def cam_to_world(xyz_cam: np.ndarray, imu_rpy_deg: np.ndarray) -> np.ndarray:
    mid = rotate_pos_axis(xyz_cam, np.array([-90.0, 0.0, -90.0]), "xyz")
    return rotate_pos_axis(mid, imu_rpy_deg, "xyz")


def world_to_cam(xyz_world: np.ndarray, imu_rpy_deg: np.ndarray) -> np.ndarray:
    inv_imu = np.array([-imu_rpy_deg[2], -imu_rpy_deg[1], -imu_rpy_deg[0]])
    mid = rotate_pos_axis(xyz_world, inv_imu, "zyx")
    return rotate_pos_axis(mid, np.array([90.0, 0.0, 90.0]), "zyx")


def simulate_impact_z(dist_h: float, pitch: float, v0: float, k_v2: float) -> float:
    sim_x = 0.0
    sim_z = 0.0
    v_x = v0 * np.cos(pitch)
    v_z = v0 * np.sin(pitch)
    t = 0.0
    while sim_x < dist_h and t < 2.0:
        v = np.hypot(v_x, v_z)
        a_x = -k_v2 * v * v_x
        a_z = -G - k_v2 * v * v_z
        sim_x += v_x * DT
        sim_z += v_z * DT
        v_x += a_x * DT
        v_z += a_z * DT
        t += DT
    return sim_z


def solve_launch_pitch(dist_h: float, z: float, v0: float, k_v2: float) -> float:
    """与车上同思路的打靶积分（拟合不用 LUT）。"""
    if dist_h < 1.5:
        return float(np.arctan2(z, dist_h))
    pitch = float(np.arctan2(z, dist_h))
    for _ in range(5):
        z_err = z - simulate_impact_z(dist_h, pitch, v0, k_v2)
        if abs(z_err) < 0.005:
            break
        pitch += z_err / max(dist_h, 0.1)
    return pitch


def solve_delta(
    xyz_cam: np.ndarray,
    imu_rpy_deg: np.ndarray,
    v_bullet: float,
    params: dict[str, float],
) -> tuple[float, float]:
    """返回 (Δyaw_rad, Δpitch_rad)，对齐 tracker solve_ballistic 输出含义。"""
    pos = np.array(
        [
            params["cam_to_gun_pos_x"],
            params["cam_to_gun_pos_y"],
            params["cam_to_gun_pos_z"],
        ]
    )
    rpy = np.array(
        [
            params["cam_to_gun_rpy_r"],
            params["cam_to_gun_rpy_p"],
            params["cam_to_gun_rpy_y"],
        ]
    )
    k_v2 = params["k_v2"]

    if v_bullet < 1e-3:
        return 0.0, 0.0

    target_w = cam_to_world(xyz_cam, imu_rpy_deg)
    offset_w = cam_to_world(pos, imu_rpy_deg)
    muzzle = target_w - offset_w
    x, y, z = muzzle
    dist_h = float(np.hypot(x, y))
    if dist_h < 0.2 or not np.isfinite(dist_h):
        return 0.0, 0.0

    pitch = solve_launch_pitch(dist_h, z, v_bullet, k_v2)
    z_aim = dist_h * np.tan(pitch)
    aim_cam = world_to_cam(np.array([x, y, z_aim]), imu_rpy_deg)

    dyaw = float(np.clip(np.arctan2(aim_cam[0], aim_cam[2]), -1.0, 1.0))
    dpitch = float(np.clip(np.arctan2(aim_cam[1], aim_cam[2]), -1.0, 1.0))
    dyaw += rpy[2] * DEG2RAD
    dpitch += rpy[1] * DEG2RAD
    return dyaw, dpitch


def load_config(path: Path) -> dict:
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_csv(path: Path) -> np.ndarray:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float, encoding="utf-8")
    if data.ndim == 0:
        data = np.array([data])
    need = ["x_cam", "y_cam", "z_cam", "imu_roll", "imu_pitch", "imu_yaw", "v_bullet"]
    missing = [k for k in need if k not in data.dtype.names]
    if missing:
        raise SystemExit(f"CSV 缺少列: {missing}")
    return data


def pack_params(cfg: dict) -> tuple[dict[str, float], list[str]]:
    values = {k: float(cfg["params"][k]["value"]) for k in PARAM_KEYS}
    free = [k for k in PARAM_KEYS if bool(cfg["params"][k].get("fit", False))]
    if not free:
        raise SystemExit("yaml 里至少要把一个参数的 fit 设为 true")
    return values, free


def main() -> None:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="弹道参数非线性最小二乘拟合")
    parser.add_argument("--config", type=Path, default=here / "fit_config.yaml")
    args = parser.parse_args()

    cfg = load_config(args.config)
    csv_path = Path(cfg["data_csv"])
    if not csv_path.is_absolute():
        csv_path = (args.config.parent / csv_path).resolve()

    rows = load_csv(csv_path)
    base, free_keys = pack_params(cfg)
    rot = np.array(
        [
            float(cfg.get("rotation_rpy_r", 0.0)),
            float(cfg.get("rotation_rpy_p", 0.0)),
            float(cfg.get("rotation_rpy_y", 0.0)),
        ]
    )
    apply_rot = bool(cfg.get("apply_rotation_rpy", True))
    bounds_cfg = cfg.get("bounds", {})

    samples = []
    for row in rows:
        xyz = np.array([row["x_cam"], row["y_cam"], row["z_cam"]], dtype=float)
        imu = np.array([row["imu_roll"], row["imu_pitch"], row["imu_yaw"]], dtype=float)
        if apply_rot:
            imu = apply_rotation_rpy(imu, rot)
        samples.append((xyz, imu, float(row["v_bullet"])))

    x0 = np.array([base[k] for k in free_keys], dtype=float)
    lo, hi = [], []
    for k in free_keys:
        b = bounds_cfg.get(k, [-np.inf, np.inf])
        lo.append(float(b[0]))
        hi.append(float(b[1]))

    def full_params(free_vec: np.ndarray) -> dict[str, float]:
        p = dict(base)
        for k, v in zip(free_keys, free_vec):
            p[k] = float(v)
        return p

    def residual(free_vec: np.ndarray) -> np.ndarray:
        p = full_params(free_vec)
        out = []
        for xyz, imu, v in samples:
            dyaw, dpitch = solve_delta(xyz, imu, v, p)
            out.extend([dyaw, dpitch])
        return np.asarray(out, dtype=float)

    print(f"数据点: {len(samples)}，本次拟合: {free_keys}")
    result = least_squares(residual, x0, bounds=(lo, hi), verbose=1)
    fitted = full_params(result.x)

    print("\n=== 结果（写回 tracker yaml 时 rpy 用度、pos 用米）===")
    for k in PARAM_KEYS:
        mark = " [本次拟合]" if k in free_keys else " [固定]"
        print(f"{k}: {fitted[k]:.8g}{mark}")
    rms = np.sqrt(np.mean(result.fun**2))
    print(f"残差 RMS (rad): {rms:.6e}  ≈ {rms * RAD2DEG:.4f} deg")


if __name__ == "__main__":
    main()
