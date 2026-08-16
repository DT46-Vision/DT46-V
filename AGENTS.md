# AGENTS.md

RoboMaster vision auto-aim stack (ROS 2 Humble, C++/Python) for team DT46. This repo is a source checkout that builds and runs on the robot PC at `/home/kie-dt46/DT46_V` (Linux); a Windows clone is for editing only. Source comments and commit messages are in Chinese — keep that style.

## Directory vs ROS package name mismatch (use package names, not dirs)

- `src/rm_DT46/` → package `rm_vision_bringup` (launch files only)
- `src/rm_vision_ros2_hik_camera/` → `hik_camera`
- `src/rm_vision_ros2_mindvision_camera/` → `mindvision_camera`
- `src/rm_vision_ros2_usb_camera/` → `usb_camera`
- `src/rm_vision_ros2_dm_imu/` → `dm_imu`
- `rm_interfaces`, `rm_detector`, `rm_serial`, `rm_tracker` match their dirs

## Build & run (on the robot)

- `colcon build --symlink-install`, then `source install/setup.bash` after `/opt/ros/humble/setup.bash`. No tests, no root CI, no pre-commit; ament lint runs only under `BUILD_TESTING`.
- Entry launches in `rm_vision_bringup`: `sentry_nav` / `hik_nav` / `hero` (HIK cam), `infantry` / `mv` (MindVision), `usb_nav`, `mv_nav`, `bag_record` (replays `~/ros2_img_msg_to_mp4/autoaim_data_bag` + detector + rqt, for offline tuning).
- `start.sh` is a deploy helper with hardcoded `WS_PATH=/home/kie-dt46/DT46_V` and `DISPLAY=:0`; launch files hardcode rqt perspective `~/DT46_V/Kielas_Vision.perspective`. Update these if the checkout path changes.

## Robot-type config variants

detector/tracker/camera each have per-robot configs (`*_infantry.yaml`, `*_hero.yaml`, `*_sentry.yaml`) plus generic `*.yaml`; each launch file picks specific variants. When adding a tunable parameter, add it to every variant used by a robot type, not just one file.

## Hardware prerequisites

- Two serial ports, expected as udev symlinks: `/dev/ttyPortMCU` (`rm_serial`, 115200) and `/dev/ttyPortIMU` (`dm_imu`, 921600). Full udev setup is in README.md; code default in `rm_serial/node.py` is `/dev/ttyACM0`.
- `hik_camera` vendors the MVS SDK in `hikSDK/` (committed to git). `mindvision_camera` requires `mvsdk/` copied in manually — it is not in the repo (`MindVisionSDK/` is gitignored).

## Pipeline & topics

camera → `rm_detector` (`/detector/armors_info` `ArmorsMsg`, debug images under `/detector/*`, subscribes `/image_raw`) → `rm_tracker` (`tracker/gimbal_control`, subscribes `/imu/rpy`, `/nav/decision`) → `rm_serial` (0x5A header + CRC16 to MCU, receives 0xA5, publishes `/nav/decision`). Messages live in `rm_interfaces`.

- `rm_detector` classifies via OpenCV DNN ONNX (`src/rm_detector/model/RM_Armor_CNN_sim.onnx`); armor classes 0–11 = B1–B7, R1–R7 (table in README).
- To add a runtime-tunable detector parameter you must edit three files together: `include/rm_detector/detector.hpp`, `src/detector.cpp`, `src/node.cpp` (declare, init, param callback) — see `src/rm_detector/thoughts.md`.

## Gotchas

- `launch/infantry.launch.py` has a live typo: `package://mindvison_camera/...` (missing "i") in `camera_info_url`.
- `src/rm_vision_ros2_mindvision_camera/.github/workflows/ros_ci.yml` is dead config (GitHub only reads root `.github/`).
- Active dev branch is `2026`; `stable` and old-year branches exist on origin.

## Code style（适用于新写的代码，存量代码未完全跟上）

代码是写给人看的，顺便能在机器上跑。

1. 命名说人话
get_image 比 proc_img 好。bbox_to_xyz_camera 比 b2x 好。
变量名不是越短越好，是越准确越好。
2. 一个函数只做一件事
80 行的函数一般可以拆成 3 个。拆不出来就说明你没想清楚流程。
3. 别写注释解释代码在干什么，写注释解释为什么这么干
# 循环 10 次 ← 废话，for i in range(10) 已经告诉你了
# 取 10 帧才检测是为了等相机曝光稳定 ← 这才值得写
4. import 顺序和分组
标准库 → 第三方 → 本项目。中间空一行。
一眼能看出依赖了哪些外部包。
5. 类型标注
输入输出标注类型不只是给 IDE 用，是给你自己用的。
三个月后回头看 def detect(image, text, mode, scale)
和 def detect(image: np.ndarray, text: str, mode: str = "phrase") -> list[dict]
后者你能直接上手，前者你得再看一遍实现。