#!/usr/bin/env python3
"""订阅视觉 / IMU / 弹速，触发一次则向 CSV 追加一行（七列）。

触发方式：
  ros2 service call /ballistic_fit_recorder/record std_srvs/srv/Trigger
反馈：
  - 服务响应 message
  - 话题 /ballistic_fit_recorder/status (std_msgs/String)
  - 节点日志
  - 可选 enable_gui:=true 弹出带按钮的小窗（状态写在按钮旁）
"""

from __future__ import annotations

import math
import os
import sys
import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

_ROOT = Path(__file__).resolve().parent.parent
_PY = Path(__file__).resolve().parent
if str(_PY) not in sys.path:
    sys.path.insert(0, str(_PY))

from csv_io import append_row, count_data_rows, prompt_choose_csv

try:
    from rm_interfaces.msg import ArmorsMsg, Decision
except ImportError as e:  # pragma: no cover
    raise SystemExit("需要已 source 的工作空间（rm_interfaces）。") from e


class BallisticFitRecorder(Node):
    def __init__(self, csv_path: Path) -> None:
        super().__init__("ballistic_fit_recorder")

        self.declare_parameter("armors_topic", "/detector/armors_info")
        self.declare_parameter("imu_topic", "/imu/rpy")
        self.declare_parameter("decision_topic", "/nav/decision")
        self.declare_parameter("default_bullet_speed", 30.0)
        self.declare_parameter("min_bullet_speed", 10.0)
        self.declare_parameter("enable_gui", False)

        self.csv_path = Path(csv_path).expanduser().resolve()
        self.default_v = float(self.get_parameter("default_bullet_speed").value)
        self.min_v = float(self.get_parameter("min_bullet_speed").value)

        self._lock = threading.Lock()
        self._armors = None
        self._imu = None  # (roll, pitch, yaw) deg
        self._v = None
        self._status = "等待数据…"

        self.status_pub = self.create_publisher(String, "~/status", 10)

        self.create_subscription(
            ArmorsMsg,
            self.get_parameter("armors_topic").value,
            self._on_armors,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Vector3Stamped,
            self.get_parameter("imu_topic").value,
            self._on_imu,
            10,
        )
        self.create_subscription(
            Decision,
            self.get_parameter("decision_topic").value,
            self._on_decision,
            qos_profile_sensor_data,
        )

        self.create_service(Trigger, "~/record", self._on_record)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(f"CSV → {self.csv_path}")
        self.get_logger().info("记录: ros2 service call /ballistic_fit_recorder/record std_srvs/srv/Trigger")

        if bool(self.get_parameter("enable_gui").value):
            self._start_gui()

    def _set_status(self, text: str, *, warn: bool = False) -> None:
        self._status = text
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)
        self._publish_status()

    def _publish_status(self) -> None:
        msg = String()
        with self._lock:
            n = count_data_rows(self.csv_path) if self.csv_path.exists() else 0
            arm_ok = self._armors is not None and len(self._armors.armors) > 0
            imu_ok = self._imu is not None
            v_ok = self._v is not None
            text = (
                f"{self._status} | 已有{n}点 | "
                f"装甲{'OK' if arm_ok else '无'} "
                f"IMU{'OK' if imu_ok else '无'} "
                f"弹速{'OK' if v_ok else '默认'}"
            )
        msg.data = text
        self.status_pub.publish(msg)

    def _on_armors(self, msg: ArmorsMsg) -> None:
        with self._lock:
            self._armors = msg

    def _on_imu(self, msg: Vector3Stamped) -> None:
        with self._lock:
            self._imu = (float(msg.vector.x), float(msg.vector.y), float(msg.vector.z))

    def _on_decision(self, msg: Decision) -> None:
        v = float(msg.bullet_speed)
        if v > self.min_v:
            with self._lock:
                self._v = v

    def _pick_armor_cam_m(self):
        """选最近装甲板，dx/dy/dz(mm) → 米。"""
        with self._lock:
            msg = self._armors
        if msg is None or not msg.armors:
            return None
        best = None
        best_d = 1e9
        for a in msg.armors:
            d = math.sqrt(a.dx * a.dx + a.dy * a.dy + a.dz * a.dz)
            if d < best_d:
                best_d = d
                best = a
        if best is None:
            return None
        return best.dx / 1000.0, best.dy / 1000.0, best.dz / 1000.0

    def try_record(self) -> tuple[bool, str]:
        xyz = self._pick_armor_cam_m()
        with self._lock:
            imu = self._imu
            v_live = self._v

        if xyz is None:
            return False, "失败：当前无装甲板"
        if imu is None:
            return False, "失败：尚无 IMU"

        v = v_live if v_live is not None else self.default_v
        note = "" if v_live is not None else f"（弹速用默认 {v:.1f}）"

        n = append_row(self.csv_path, xyz[0], xyz[1], xyz[2], imu[0], imu[1], imu[2], v)
        return True, f"OK 已写入第 {n} 点{note} | {self.csv_path.name}"

    def _on_record(self, _req, resp: Trigger.Response):
        ok, msg = self.try_record()
        resp.success = ok
        resp.message = msg
        self._set_status(msg, warn=not ok)
        return resp

    def _start_gui(self) -> None:
        try:
            import tkinter as tk
        except ImportError:
            self.get_logger().error("无 tkinter，无法开 GUI")
            return

        def ui_main() -> None:
            root = tk.Tk()
            root.title("弹道标定采数")
            root.attributes("-topmost", True)
            status = tk.StringVar(value=self._status)
            path_var = tk.StringVar(value=str(self.csv_path))

            tk.Label(root, textvariable=path_var, wraplength=420, justify="left").pack(padx=8, pady=4)

            def on_click() -> None:
                ok, msg = self.try_record()
                status.set(msg)
                self._set_status(msg, warn=not ok)

            btn = tk.Button(root, text="记录一点", command=on_click, width=20, height=2)
            btn.pack(padx=8, pady=8)
            lbl = tk.Label(root, textvariable=status, wraplength=420, justify="left", fg="#0a0")
            lbl.pack(padx=8, pady=4)

            def poll() -> None:
                # 同步话题状态到副行
                with self._lock:
                    arm_ok = self._armors is not None and len(self._armors.armors) > 0
                    imu_ok = self._imu is not None
                path_var.set(
                    f"{self.csv_path}\n装甲{'有' if arm_ok else '无'} | IMU{'有' if imu_ok else '无'} | 点按上方按钮"
                )
                root.after(300, poll)

            poll()
            root.mainloop()

        threading.Thread(target=ui_main, daemon=True).start()


def main() -> None:
    # 先选 CSV（可新建），再进 ROS，避免和 spin 抢终端输入
    data_dir = _ROOT / "data"
    csv_env = os.environ.get("CSV_PATH", "").strip()
    if csv_env:
        csv_path = Path(csv_env).expanduser()
        if not csv_path.is_absolute():
            csv_path = (_ROOT / csv_path).resolve()
        from csv_io import ensure_csv

        ensure_csv(csv_path)
        print(f"使用 CSV_PATH: {csv_path}")
    else:
        csv_path = prompt_choose_csv(data_dir, allow_new=True)

    rclpy.init()
    node = BallisticFitRecorder(csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
