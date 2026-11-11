#!/usr/bin/env python3
"""把 /ekf/debug 保存成 PlotJuggler / Excel 可直接打开的 CSV。"""

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from rm_interfaces.msg import EkfDebug


def flatten_scalar_fields(msg) -> tuple[list[str], list[float]]:
    """按消息定义顺序取出标量字段；header 跳过，时间戳单独成列。"""
    names: list[str] = []
    values: list[float] = []
    for field_name, field_type in msg.get_fields_and_field_types().items():
        if field_name == "header":
            continue

        value = getattr(msg, field_name)
        if isinstance(value, bool):
            names.append(field_name)
            values.append(float(int(value)))
        elif isinstance(value, (int, float)):
            names.append(field_name)
            values.append(float(value))
        else:
            raise TypeError(
                f"字段 {field_name} 类型为 {field_type}，本脚本只支持标量字段"
            )
    return names, values


class EkfDebugCsvWriter(Node):
    def __init__(self, topic: str, output: Path) -> None:
        super().__init__("ekf_debug_csv_writer")
        self._output = output
        self._file = output.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._header_written = False
        self._rows_saved = 0
        self._last_flush = time.monotonic()

        # 与发布端保持一致用可靠 QoS，避免调参时漏掉关键帧
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self._sub = self.create_subscription(EkfDebug, topic, self._on_msg, qos)
        self.get_logger().info(f"正在保存 {topic} -> {output}")

    @property
    def rows_saved(self) -> int:
        return self._rows_saved

    def _on_msg(self, msg: EkfDebug) -> None:
        names, values = flatten_scalar_fields(msg)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        recv_unix = time.time()

        if not self._header_written:
            self._writer.writerow(["stamp", "recv_unix", *names])
            self._header_written = True

        self._writer.writerow([f"{stamp:.9f}", f"{recv_unix:.9f}", *values])
        self._rows_saved += 1

        # 每秒落盘一次：进程被杀也最多丢最近 1 秒数据
        now = time.monotonic()
        if now - self._last_flush >= 1.0:
            self._file.flush()
            self._last_flush = now

    def close(self) -> None:
        if not self._file.closed:
            self._file.flush()
            self._file.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="把 /ekf/debug 保存为 CSV")
    parser.add_argument("--topic", default="/ekf/debug", help="订阅的话题名")
    parser.add_argument(
        "-o",
        "--output",
        default=None,
        help="输出 CSV 路径，默认 ekf_debug_<时间戳>.csv",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output = (
        Path(args.output)
        if args.output
        else Path(f"ekf_debug_{datetime.now():%Y%m%d_%H%M%S}.csv")
    )
    output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = EkfDebugCsvWriter(args.topic, output)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        # 用 print 而不是 rosout：外部 SIGTERM 时上下文可能已失效
        if node.rows_saved == 0:
            print("没有收到任何消息，CSV 为空")
        else:
            print(f"已保存 {node.rows_saved} 行 -> {output}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
