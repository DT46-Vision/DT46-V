---
name: dt46
description: >-
  DT46 RoboMaster vision map for ROS 2 Humble: package names, launch files,
  per-robot yaml, and cheap build or search limits. Use when compiling,
  launching, or editing rm_tracker, rm_detector, rm_serial, dm_imu, or camera configs.
---

# DT46

先缩小范围再读代码。一次只打开会改的文件。

## 不要读

`hikSDK/`、`mvsdk/`、`build/`、`install/`、`*.so`、`*.onnx`。SDK 头文件只在编译错误点名某个符号时，用 grep 取匹配行。

根 README 和 `src/rm_tracker/README.md` 过时。tracker 以 `src/rm_tracker/src` 为准。检测器是 OpenCV DNN（`cv::dnn::readNetFromONNX`），不是 ONNX Runtime。模型：`src/rm_detector/model/RM_Armor_CNN_sim.onnx`。

## 编译

每个 shell 先 `source /opt/ros/humble/setup.bash` 一次。只编改过的包：

```bash
colcon build --packages-select <pkg> --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install
source install/setup.bash
```

改了 `rm_interfaces` 的 msg，先编它再编依赖它的包。用户没要求时不要 `rm -rf build install log`，也不要全量编译。C++ 包带 `-O3 -march=native`，编译慢。没有测试。

## 包

| 目录 | 包名 | 语言 |
|---|---|---|
| `src/rm_DT46` | `rm_vision_bringup` | launch |
| `src/rm_tracker` | `rm_tracker` | C++ `tracker_core` + `rm_tracker_node` |
| `src/rm_detector` | `rm_detector` | C++ |
| `src/rm_interfaces` | `rm_interfaces` | msg |
| `src/rm_serial` | `rm_serial` | Python |
| `src/rm_vision_ros2_dm_imu` | `dm_imu` | Python |
| hik / mindvision / usb camera | 各自 package.xml 的 `<name>` | C++ |

Launch 用 `rm_vision_bringup`，不用目录名 `rm_DT46`。

| 机器人 | launch | 配置 |
|---|---|---|
| 哨兵 | `hik.launch.py` / `sentry.launch.py` | `detector_params_sentry.yaml` + `tracker_params_sentry.yaml` + `sentinel_camera_params.yaml` |
| 步兵 | `mv.launch.py` / `infantry.launch.py` | `*_infantry.yaml` |
| 英雄 | `hero.launch.py` | `*_hero.yaml` |
| 只看视觉 | `*_nav.launch.py` | 无串口、无 IMU |
| 回放 | `bag_record.launch.py` | rosbag |

串口写死：`/dev/ttyPortMCU` 115200，`/dev/ttyPortIMU` 921600。`start.sh` 里的家目录路径是另一台机器的，不要照搬。

## 其他

- 没有 CI、linter。`build/` 或 `install/` 不存在时，先编 `rm_interfaces`。
- `build/` 可能还是旧的 Python tracker 布局。只有用户要求，或这个包明显是旧布局编出来的，才清空 `build/`、`install/`、`log/` 再编。
- 提交说明用中文短句，形如「修改xxx」。
- 相机 SDK 已在仓库里，不要再装外部 SDK。
- `thoughts.md`：改动尽量小，符合工程规范。
