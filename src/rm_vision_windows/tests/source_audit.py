"""Verify imported ROS files and the protected tracker against their recorded sources."""

import argparse
import ast
import json
from pathlib import Path
import subprocess

from rm_vision_windows.aim import ROOT


ROS_SOURCE = "96bc416"
TRACKER_SOURCE = "3c41f0b02a05bf06ad28ca8440bc8099545498ab"
ROS_PATHS = ["src/rm_DT46", "src/rm_serial", "src/rm_vision_ros2_dm_imu", "src/rm_interfaces", "start.sh",
    "src/rm_detector/launch", "src/rm_tracker/launch", "src/rm_detector/src/node.cpp"]
for camera in ("hik", "mindvision", "usb"):
    ROS_PATHS.extend(f"src/rm_vision_ros2_{camera}_camera/{part}" for part in ("launch", "package.xml", "README.md"))
TRACKER_PATHS = [f"src/rm_tracker/{path}" for path in ("src/tracker.cpp", "include/rm_tracker/tracker.hpp",
    "src/ekf.cpp", "include/rm_tracker/ekf.hpp", "src/node.cpp")]


def git(*arguments: str) -> str:
    return subprocess.check_output(["git", "-C", str(ROOT), *arguments], text=True, encoding="utf-8").strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    options = parser.parse_args()
    report = {"passed": False, "ros_source": git("rev-parse", ROS_SOURCE),
              "tracker_source": TRACKER_SOURCE, "vision_head": git("rev-parse", "HEAD")}
    try:
        report["ros_diff"] = git("diff", "--name-only", ROS_SOURCE, "--", *ROS_PATHS).splitlines()
        report["tracker_diff"] = git("diff", "--name-only", TRACKER_SOURCE, "--", *TRACKER_PATHS).splitlines()
        files = git("ls-tree", "-r", "--name-only", ROS_SOURCE, "--", *ROS_PATHS).splitlines()
        report["ros_files"] = files
        report["python_syntax_files"] = 0
        for name in files:
            if name.endswith(".py"):
                ast.parse((ROOT / name).read_text(encoding="utf-8-sig"), filename=name)
                report["python_syntax_files"] += 1
        if report["ros_diff"] or report["tracker_diff"]:
            raise AssertionError("ROS import or protected tracker differs from its recorded source")
        report["passed"] = True
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
    options.output.parent.mkdir(parents=True, exist_ok=True)
    options.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "ros_files"}, indent=2))
    return int(not report["passed"])


if __name__ == "__main__":
    raise SystemExit(main())
