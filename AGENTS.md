# AGENTS.md — DT46-V (RoboMaster Vision Auto-Aim, ROS 2 Humble)

## Build

```bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install
source install/setup.bash
```

- `--symlink-install` is **required** for Python packages (`rm_serial`, `dm_imu`) so edits take effect without rebuild.
- No `colcon.meta` file — all build flags are on the command line.

## Package / Directory Name Mismatch

The directory `src/rm_DT46/` declares package name **`rm_vision_bringup`** in its `package.xml`. All ROS 2 commands (launch, colcon) use the package name, not the directory name:

```bash
ros2 launch rm_vision_bringup hik.launch.py   # sentry
ros2 launch rm_vision_bringup mv.launch.py    # infantry
```

## Robot-Type Config Selection

Detector and tracker have per-robot config variants. The **launch file** picks the right one — do not edit the wrong yaml:

| Robot   | Detector Config                    | Tracker Config                    |
|---------|-----------------------------------|-----------------------------------|
| Sentry  | `detector_params_sentry.yaml`     | `tracker_params_sentry.yaml`      |
| Hero    | `detector_params_hero.yaml`       | `tracker_params_hero.yaml`        |
| Infantry| `detector_params_infantry.yaml`   | `tracker_params_infantry.yaml`    |
| Default | `detector_params.yaml`            | `tracker_params.yaml`             |

## Tracker Is C++, Not Python

The README describes `rm_tracker` as Python, but it was **rewritten in C++** (`-O3 -march=native`). A stale `rm_tracker/modules/__pycache__/` directory remains — ignore it.

## External Dependencies

- **`rm_opencv_aim`**: Declared as a dep of `rm_vision_bringup` but **not in this workspace**. Likely installed system-wide or missing. Do not add it to this repo.
- **Camera SDKs**: The `hik_camera` and `mindvision_camera` packages require vendor SDKs (MVS, Hik SDK) installed outside the workspace.

## Custom Messages

All messages live in `src/rm_interfaces/msg/`. No `srv/` exists. Key types:
- `ArmorsMsg` / `ArmorInfo` — detection output (3D coords + yaw per armor)
- `GimbalControl` — gimbal command (pitch, yaw, can_fire)
- `Decision` — referee game state (color, bullet_speed, HP, etc.)

If you add/modify messages, rebuild `rm_interfaces` first — packages depending on it need the generated headers.

## ONNX Model

`src/rm_detector/model/RM_Armor_CNN_sim.onnx` — the CNN classifier. Installed to `share/rm_detector/model/`. All detector configs reference it by filename.

## No Tests

There are no test files in the workspace. `package.xml` files declare test dependencies (pytest, linters) but no test targets exist. `colcon test` will find nothing.

## Serial Ports

Requires udev rules for fixed symlinks (`/dev/ttyPortIMU`, `/dev/ttyPortMCU`). After configuring, **physically replug the USB cables** or symlinks won't appear.

## Start Script

`start.sh` hardcodes workspace path `/home/kie-dt46/DT46_V` — this differs from the current path `/home/dt46/DT46-V`. Update if you use it.

## Known Stale Reference

`src/rm_DT46/launch/mv_nav.launch.py` references `~/ros_vision/Kielas_Vision.perspective` instead of `~/DT46_V/Kielas_Vision.perspective` (which is what other launch files use).
