"""七列标定 CSV 读写（无 ROS 依赖）。"""

from __future__ import annotations

import csv
from pathlib import Path

HEADER = ["x_cam", "y_cam", "z_cam", "imu_roll", "imu_pitch", "imu_yaw", "v_bullet"]


def ensure_csv(path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or path.stat().st_size == 0:
        with path.open("w", newline="", encoding="utf-8") as f:
            csv.writer(f).writerow(HEADER)


def count_data_rows(path: Path) -> int:
    path = Path(path)
    if not path.exists():
        return 0
    with path.open(encoding="utf-8") as f:
        lines = [ln for ln in f.read().splitlines() if ln.strip()]
    if not lines:
        return 0
    if lines[0].startswith("x_cam"):
        return max(0, len(lines) - 1)
    return len(lines)


def list_data_csvs(data_dir: Path) -> list[Path]:
    data_dir = Path(data_dir)
    if not data_dir.is_dir():
        return []
    return sorted(p for p in data_dir.glob("*.csv") if p.is_file())


def prompt_choose_csv(data_dir: Path, *, allow_new: bool) -> Path:
    """交互选择 data/ 下的 csv；allow_new 时可新建。"""
    data_dir = Path(data_dir)
    data_dir.mkdir(parents=True, exist_ok=True)
    files = list_data_csvs(data_dir)

    print(f"\n目录: {data_dir}")
    if files:
        print("已有 CSV：")
        for i, p in enumerate(files, 1):
            n = count_data_rows(p)
            print(f"  {i}. {p.name}  ({n} 个数据点)")
    else:
        print("（当前没有 .csv）")

    if allow_new:
        print("  0. 新建 CSV 文件")
    if not files and not allow_new:
        raise SystemExit(f"data/ 下没有 CSV，请先采数: {data_dir}")

    while True:
        raw = input("请输入序号: ").strip()
        if allow_new and raw == "0":
            name = input("新文件名（直接回车=calib.csv）: ").strip() or "calib.csv"
            if not name.endswith(".csv"):
                name += ".csv"
            path = data_dir / name
            ensure_csv(path)
            print(f"将写入: {path}")
            return path
        if raw.isdigit():
            idx = int(raw)
            if 1 <= idx <= len(files):
                path = files[idx - 1]
                print(f"已选择: {path}")
                return path
        print("无效序号，请重试。")


def append_row(
    path: Path,
    x_cam: float,
    y_cam: float,
    z_cam: float,
    imu_roll: float,
    imu_pitch: float,
    imu_yaw: float,
    v_bullet: float,
) -> int:
    """追加一行，返回当前数据点序号（从 1 起）。"""
    path = Path(path)
    ensure_csv(path)
    with path.open("a", newline="", encoding="utf-8") as f:
        csv.writer(f).writerow(
            [
                f"{x_cam:.6f}",
                f"{y_cam:.6f}",
                f"{z_cam:.6f}",
                f"{imu_roll:.6f}",
                f"{imu_pitch:.6f}",
                f"{imu_yaw:.6f}",
                f"{v_bullet:.6f}",
            ]
        )
    return count_data_rows(path)
