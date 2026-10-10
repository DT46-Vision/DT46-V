# rm_vision_bringup

整车/分场景的启动组合包。**只有 launch 文件，没有可执行节点**，负责按兵种把相机驱动、IMU、串口、检测、追踪和 rqt 调试界面组合起来，并指定各自的参数文件。

> 目录名是 `src/rm_DT46`，但 `package.xml` 里声明的是 `<name>rm_vision_bringup</name>`。所有命令都用 **`rm_vision_bringup`**，不要用 `rm_DT46`。

## 包内容

```
launch/
├── hik.launch.py        # 海康相机 + 串口 + IMU + 检测 + 追踪 + rqt（哨兵参数，默认整机入口）
├── mv.launch.py         # 迈德威视相机 + 串口 + IMU + 检测 + 追踪 + rqt（步兵参数）
├── hero.launch.py       # 英雄专用：海康相机 + 全套节点，参数写死
├── sentry.launch.py     # 哨兵专用：海康相机 + 全套节点，参数写死
├── infantry.launch.py   # 步兵：迈德威视相机 + 全套节点
├── hik_nav.launch.py    # 纯视觉：海康相机 + IMU + 检测 + 追踪（无串口）
├── mv_nav.launch.py     # 纯视觉：迈德威视相机 + 检测 + 追踪（无串口/IMU）
├── usb_nav.launch.py    # 纯视觉：USB 相机 + IMU + 检测 + 追踪（无串口）
└── bag_record.launch.py # rosbag 回放 + 检测 + rqt（追踪节点默认注释掉）
```

## 各 launch 启动的节点

| launch | 相机 | 串口 | IMU | 检测 | 追踪 | rqt | 算法参数来源 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `hik.launch.py` | `hik_camera` | 是 | 是 | 是 | 是 | 是 | 哨兵：`detector/tracker_params_sentry.yaml` |
| `mv.launch.py` | `mv_camera`（mindvision_camera） | 是 | 是 | 是 | 是 | 是 | 步兵：`detector/tracker_params_infantry.yaml` |
| `infantry.launch.py` | `mv_camera` | 是 | 是 | 是 | 是 | 是 | 步兵同上 |
| `hero.launch.py` | `hik_camera` | 是 | 是 | 是 | 是 | 是 | 英雄：`*_params_hero.yaml` + `hero_camera_params/info.yaml` |
| `sentry.launch.py` | `hik_camera` | 是 | 是 | 是 | 是 | 是 | 哨兵：`*_params_sentry.yaml` + `sentinel_camera_params/info.yaml` |
| `hik_nav.launch.py` | `hik_camera` | 否 | 是 | 是 | 是 | 是 | 哨兵算法参数 |
| `mv_nav.launch.py` | `mv_camera` | 否 | 否 | 是 | 是 | 是 | 通用：`detector_params.yaml` + `tracker_params.yaml` |
| `usb_nav.launch.py` | `usb_camera` | 否 | 是 | 是 | 是 | 是 | 通用算法参数 + `usb_camera` 的相机参数 |
| `bag_record.launch.py` | rosbag 回放 | 否 | 否 | 是 | 否（注释掉） | 是 | 通用参数 |

每个 launch 末尾都会拉起 `rqt_gui`，加载 `~/DT46_V/Kielas_Vision.perspective`（见"注意事项"第 2 条）。

## launch 参数

### 带完整 launch argument 的（hik / hik_nav / mv / infantry / mv_nav / usb_nav）

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `params_file` | `hik_camera` 或 `mindvision_camera` 的 `config/camera_params.yaml` | 相机参数（曝光、增益等） |
| `camera_info_url` | `package://<相机包>/config/camera_info.yaml` | 相机内参文件 |
| `use_sensor_data_qos` | `false` | 相机图像是否用 SensorDataQoS 发布 |
| `detector_params_sentry_file` / `detector_params_infantry_file` / `detector_params_file` | 对应兵种的检测参数 | `rm_detector` 参数文件 |
| `tracker_params_sentry_file` / `tracker_params_infantry_file` / `tracker_params_file` | 对应兵种的追踪参数 | `rm_tracker` 参数文件 |
| `serial_params_file` | `rm_serial/config/rm_serial_params.yaml` | 串口参数（`*_nav` 里没有这一项） |
| `dm_imu_params_file` | `dm_imu/config/dm_imu_params.yaml` | IMU 参数（`mv_nav` 里没有这一项） |
| `camera_params_file` | `usb_camera/config/camera_params.yaml` | 仅 `usb_nav.launch.py` |

用法示例：

```bash
ros2 launch rm_vision_bringup hik.launch.py camera_info_url:=package://hik_camera/config/hero_camera_info.yaml
ros2 launch rm_vision_bringup mv.launch.py detector_params_infantry_file:=/abs/path/detector_params.yaml
```

### 参数写死的（hero / sentry）

`hero.launch.py` 与 `sentry.launch.py` **没有暴露 launch argument**，参数路径全部写死在文件里：

- 英雄：`hik_camera/config/hero_camera_params.yaml` + `package://hik_camera/config/hero_camera_info.yaml`、`detector_params_hero.yaml`、`tracker_params_hero.yaml`。
- 哨兵：`hik_camera/config/sentinel_camera_params.yaml` + `package://hik_camera/config/sentinel_camera_info.yaml`、`detector_params_sentry.yaml`、`tracker_params_sentry.yaml`。
- 两者都用 `rm_serial/config/rm_serial_params.yaml` 与 `dm_imu/config/dm_imu_params.yaml`。

要临时改，只能编辑 launch 文件，或者改用 `hik.launch.py` 并传参。

### bag_record.launch.py

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `bag_file` | `~/ros2_img_msg_to_mp4/autoaim_data_bag` | 要回放的 rosbag 路径 |
| `detector_params_file` | `rm_detector/config/detector_params.yaml` | 检测参数 |
| `tracker_params_file` | `rm_tracker/config/tracker_params.yaml` | 追踪参数（当前未使用，见注意事项） |

行为：`ros2 bag play <bag_file> --loop` + `rm_detector` + `rqt_gui`；回放用 `--loop`，不加 `--clock`（`rm_detector` 使用图像自带时间戳）。

## 使用方式

### 编译与加载环境

```bash
source /opt/ros/humble/setup.bash
cd ~/DT46-V
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install
source install/setup.bash
```

本包依赖 `rm_interfaces`、`rm_detector`、`rm_tracker`、`rm_serial`、`dm_imu`、`hik_camera`、`mindvision_camera`、`usb_camera`。增量开发时可用：

```bash
colcon build --packages-up-to rm_vision_bringup
```

### 启动

```bash
# 整机（含串口/IMU，实车）
ros2 launch rm_vision_bringup hik.launch.py      # 哨兵（海康）
ros2 launch rm_vision_bringup mv.launch.py       # 步兵（迈德威视）
ros2 launch rm_vision_bringup hero.launch.py     # 英雄（参数写死）
ros2 launch rm_vision_bringup sentry.launch.py   # 哨兵（参数写死）

# 纯视觉（不连下位机，调算法/录数据）
ros2 launch rm_vision_bringup hik_nav.launch.py
ros2 launch rm_vision_bringup mv_nav.launch.py
ros2 launch rm_vision_bringup usb_nav.launch.py

# 数据集回放
ros2 launch rm_vision_bringup bag_record.launch.py bag_file:=/path/to/bag
```

建议在 `~/.bashrc` 里加别名：

```bash
alias rmhiknav='ros2 launch rm_vision_bringup hik_nav.launch.py'
alias rmhik='ros2 launch rm_vision_bringup hik.launch.py'
alias rmmv='ros2 launch rm_vision_bringup mv.launch.py'
```

仓库根目录的 `start.sh` 等价于 "source ROS + source install + `export DISPLAY=:0`"，但里面的 `WS_PATH` 写死为 `/home/kie-dt46/DT46_V`，换机器或改目录后需要修改。

## 依赖与前置条件

- `package.xml` 声明的依赖：`ament_cmake`、`rm_opencv_aim`、`rm_interfaces`（`rm_opencv_aim` 见注意事项第 1 条）。
- 实际运行时还依赖：`hik_camera`、`mindvision_camera`、`usb_camera`、`rm_detector`、`rm_tracker`、`rm_serial`、`dm_imu`、`rqt_gui`。
- 硬件前置（整机 launch）：
  - 相机：海康工业相机（`hik` / `hero` / `sentry` / `hik_nav`）、迈德威视（`mv` / `infantry` / `mv_nav`）、USB 摄像头（`usb_nav`）。
  - 串口：`/dev/ttyPortMCU` @115200（`rm_serial`），需要 udev 规则，见 `rm_serial` README。
  - IMU：`/dev/ttyPortIMU` @921600（`dm_imu`）。
- 相机内参文件必须与当前分辨率/镜头匹配（`camera_info.yaml` / `hero_camera_info.yaml` / `sentinel_camera_info.yaml`），否则测距会有系统性偏差。

## 注意事项 / 已知问题

1. **`package.xml` 依赖的 `rm_opencv_aim` 在工作空间里不存在**（历史遗留）。`colcon build` 不受影响，但 `rosdep install --from-paths src` 会因找不到该依赖报错。
2. **rqt 布局路径与本机实际路径不符**：所有 launch 都写 `os.path.expanduser("~/DT46_V/Kielas_Vision.perspective")`，而本仓库在 `~/DT46-V`（布局文件在仓库根目录）。路径不存在时 rqt 会以空白布局启动（不崩溃，但看不到预设界面）。修正方式是做一个软链接 `~/DT46_V -> ~/DT46-V`，或改 launch 里的路径。
3. **`bag_record.launch.py` 里追踪节点被注释掉**：只跑检测；`tracker_params_file` 参数虽然声明了但没被使用。想联调追踪需要自己取消注释。另外它的 rqt 布局用的是相对路径 `Kielas_Vision.perspective`（`expanduser` 不会把相对路径变成仓库根），会按"当前工作目录"去找。
4. **`hero.launch.py` / `sentry.launch.py` 无法用命令行覆盖参数**：路径硬编码，换文件必须改 launch。
5. **`infantry.launch.py` 与 `mv.launch.py` 内容几乎相同**（都是迈德威视 + 步兵参数），维护时注意同步修改。
6. **纯视觉 launch 没有 `/nav/decision`**：`hik_nav` / `mv_nav` / `usb_nav` 不启动 `rm_serial`，`rm_tracker` 的 `follow_decision` 必须保持 `false`（通用/哨兵参数已如此），否则颜色不会跟随任何输入。
7. **`mv_nav.launch.py` 不启动 IMU**：`rm_tracker` 收不到 `/imu/rpy` 时完全不追踪（`armors_cb` 直接丢弃数据）。用这个 launch 联调追踪时，需要另外启动 `dm_imu` 或自己发 `/imu/rpy`。
8. **`use_sensor_data_qos` 默认 false**：相机图像按 RELIABLE 发布，带宽/延迟更高；对帧率敏感时可设为 true（`rm_detector` 订阅用的是 SensorDataQoS，两种都能收）。
9. **launch 里的节点名会覆盖节点内部名字**：例如迈德威视相机在 ROS 图里叫 `mv_camera`（不是可执行名 `mindvision_camera_node`），排查话题时以 `ros2 node list` 为准。
10. **`usb_nav.launch.py` 目前起不来（已实测）**：`usb_camera/config/camera_params.yaml` 里写的是 `fps: 240`（YAML 整数），而 `usb_camera` 节点声明的 `fps` 是 double，rclcpp 会抛 `InvalidParameterTypeException` 并让进程直接终止。把该行改成 `fps: 240.0` 即可启动，详见 `usb_camera` README 注意事项第 1 条。
