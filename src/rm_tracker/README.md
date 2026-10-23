# rm_tracker

装甲板追踪与弹道解算节点（C++ / ROS 2 Humble）。消费 `rm_detector` 的装甲板观测、`dm_imu` 的姿态和相机内参，完成

相机系 → 世界系坐标变换 → 目标选择/装甲板跳变处理 → 9 维 EKF 状态估计 → 反小陀螺判定 → 空气阻力弹道解算 → 输出云台 `yaw/pitch` 与 `can_fire`。

代码分为两层：`tracker_core`（`tracker.cpp` / `ekf.cpp` / `rm_tf_tools.cpp`，纯算法）与 `rm_tracker_node`（`node.cpp`，ROS 通信、线程、渲染）。

无 ROS 接入见 [standalone](../../standalone/README.md)。`RmTF` 可接收直接光学旋转，并通过 `set_camera_info(..., false)` 保留标定主点；原 ROS 调用保持原默认行为。

## 节点与可执行文件

| 节点名（ROS 图中的名字） | 可执行文件 | 作用 |
| --- | --- | --- |
| `rm_tracker` | `rm_tracker_node` | 追踪 + 预测 + 弹道解算，输出云台控制量与调试图像 |

- 节点名硬编码为 `Node("rm_tracker")`，YAML 根键必须是 `rm_tracker:`。
- 用 `MultiThreadedExecutor` 起节点，另有独立 `processing_worker` 线程做解算；装甲板队列长度恒为 1（新数据到达即清空旧数据），保证永远只算最新一帧。

## 话题接口

| 方向 | 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- | --- |
| 订阅 | `/detector/armors_info` | `rm_interfaces/msg/ArmorsMsg` | SensorDataQoS | 装甲板观测（相机系 mm / deg），追踪的唯一输入 |
| 订阅 | `/imu/rpy` | `geometry_msgs/msg/Vector3Stamped` | 默认 QoS，队列 10 | 云台姿态 RPY，**单位：度**（与 `dm_imu` 默认输出一致） |
| 订阅 | `/camera_info` | `sensor_msgs/msg/CameraInfo` | SensorDataQoS | 相机内参，仅第一帧生效 |
| 订阅 | `/nav/decision` | `rm_interfaces/msg/Decision` | SensorDataQoS | 裁判系统决策（只用到 `color` 与 `bullet_speed`） |
| 订阅 | `/image_raw` | `sensor_msgs/msg/Image` | SensorDataQoS | 仅用于渲染调试图，`display: false` 时回调直接 return |
| 发布 | `/tracker/gimbal_control` | `rm_interfaces/msg/GimbalControl` | SensorDataQoS | 云台 yaw/pitch（度）与 `can_fire`，`frame_id = tracking_frame` |
| 发布 | `/tracker/tracking_state_img` | `sensor_msgs/msg/Image` | SensorDataQoS | 状态可视化，仅 `display: true` |
| 发布 | `/tracker/ballistic_img` | `sensor_msgs/msg/Image` | SensorDataQoS | HUD（距离/角度差/旋转状态/弹速等），仅 `display: true` |

要点：

- `/imu/rpy` 必须是**度**。`dm_imu` 的 `publish_rpy_in_degree` 默认 true，所以默认兼容；若把它改成 false，追踪会完全错乱。
- 没有 IMU 就不会追踪：`armors_cb` 在收到第一帧 `/imu/rpy` 之前直接丢弃装甲板数据（`imu_initialized_` 门控）。只回放 bag 时需要自己补一个 `/imu/rpy` 发布者。
- `can_fire` 在 `GimbalControl` 里是 `int32`，节点写 0/1。
- 发出的 yaw/pitch 经过 EMA 平滑：`smooth = 0.25 * raw + 0.75 * smooth`（`can_fire == false` 时重置为下一帧直接对齐）。

## 数据流与算法

1. **颜色过滤 + 坐标变换**（`node.cpp`）：`target_color == 0` 只保留 `armor_id >= 6`（红方），`== 1` 只保留 `armor_id < 6`（蓝方）。相机系坐标 `dx/dy/dz`（mm → m）经 `RmTF::cam_to_world()` 结合 IMU RPY 转到世界系；观测 yaw 取反后与云台 yaw 相加得到世界系绝对 yaw。
2. **时间步长**：`dt` 由 `ArmorsMsg.header.stamp` 差分得到；`dt <= 1ms` 时使用 10ms，`dt > 100ms` 时截断到 100ms，其余使用实际间隔。
3. **目标选择与跳变**（`tracker.cpp`）：先用位置消歧观测 yaw，同一消歧结果用于角度匹配、换板和 EKF 更新。当前位置、角差分别通过 `max_match_distance` 和 `max_match_yaw_diff` 时直接更新；否则仅在观测更接近其他预测板、且换板冷却结束时处理跳变。
4. **EKF**：9 维状态 `[xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r]`，观测为 4 维 `[x, y, z, yaw]`；Q/R 由 `ekf_QR_*` 参数在线计算，半径 `r` 用 `radius_r_min/max` 限幅。
5. **状态机**：`LOST → DETECTING`（连续 `tracking_thres` 帧）`→ TRACKING → TEMP_LOST → LOST`（连续 `lost_thres` 帧）。状态切换会打印到终端。
6. **反小陀螺**：同向换板连续两次进入 SPIN；没有继续换板时，结合板运动速度计时退出。每次平移判断先淘汰距当前时刻超过 0.6 s 的双板车心样本；剩余样本跨度不足 0.4 s 时按原地处理，切换仍有滞回。SPIN 使用到达时刻的四板预测，在车心正前等板；判为平移时才外推车心，否则忽略残余线速度。
7. **弹道解算**：`build_ballistic_lut()` 用 `bullet_speed` 与 `k_v2`（平方空气阻力 `a = -k·v²`）预建 25×15 查找表（距离 0.1–12 m 步长 0.5，高度 −3.5–3.5 m 步长 0.5），运行时按目标距离/高度查表得到 pitch 补偿；同时补偿 `system_delay`（弹丸飞行时间 + 系统延迟）与相机→枪口外参 `cam_to_gun_*`。
8. **开火判定**：目标距离 ≤ `shootable_dist`，yaw 误差位于装甲左右边缘角窗内，pitch 误差通过动态门限。SPIN 额外检查所选装甲在弹丸到达时刻是否位于正对窗口；交替半径和板高随选板同步，车心不可用时关闭发弹许可。`controller_delay` 只影响瞄准点外推，不改变选板的到达时刻。

## 参数

参数文件：`config/tracker_params.yaml`（通用）、`tracker_params_hero.yaml`、`tracker_params_infantry.yaml`、`tracker_params_sentry.yaml`。

| 参数 | 类型 | 代码 declare 默认值 | 含义 |
| --- | --- | --- | --- |
| `target_color` | int | `0` | 要打的颜色：0 = 红（保留 id ≥ 6），1 = 蓝（保留 id < 6） |
| `follow_decision` | bool | `false` | true 时用 `/nav/decision` 的 `color` 覆盖 `target_color`（覆盖会重置状态机为 LOST） |
| `bullet_speed` | double | `28.0` | 弹速（m/s）；`/nav/decision` 的 `bullet_speed > 10` 时也会自动同步并重建弹道表 |
| `display` | bool | `false` | 是否发布两路调试图 |
| `display_fps_limit` | bool | `true` | 渲染限速 30 FPS |
| `text_size` | double | `1.0` | 调试图文字缩放 |
| `rotation_rpy_r/p/y` | double | `0 / 0 / −180` | IMU → 相机的轴固定旋转修正（度） |
| `cam_to_gun_pos_x/y/z` | double | `0 / 0.125 / 0` | 相机 → 枪口平移外参（m） |
| `cam_to_gun_rpy_r/p/y` | double | `0 / 0 / 0` | 相机 → 枪口旋转外参（度） |
| `max_match_distance` | double | `0.2` | 观测关联的最大位置偏差（m） |
| `max_match_yaw_diff` | double | `57.0` | 判定装甲板跳变的 yaw 差阈值（度） |
| `jump_cooldown_max` | int | `20` | 跳变后的冷却帧数 |
| `tracking_thres` | int | `5` | 连续命中多少帧进入 TRACKING |
| `lost_thres` | int | `20` | 连续丢失多少帧进入 LOST |
| `dist_tol` | double | `0.15` | 预处理阶段两块板深度接近的容差（m） |
| `ekf_QR_q_xyz` | double | `2.5` | 位置过程噪声谱密度（越大越信任观测） |
| `ekf_QR_q_yaw` | double | `15.0` | 角度过程噪声谱密度 |
| `ekf_QR_q_r` | double | `0.05` | 半径过程噪声谱密度 |
| `ekf_QR_r_xyz_factor` | double | `0.05` | PnP 位置噪声：距离百分比（0.2 = 0.2%），`R = (factor*dist + 5mm)²` |
| `ekf_QR_r_yaw` | double | `0.02` | 角度观测噪声方差 |
| `ekf_QR_stable_dist` | double | `1.5` | 动态调整 Q/R 的参考距离（m） |
| `radius_r_max` / `radius_r_min` | double | `0.4` / `0.12` | 车辆半径限幅（m） |
| `system_delay` | double | `0.1` | 系统延迟补偿（s） |
| `k_v2` | double | `0.019` | 平方空气阻力系数 |
| `shootable_dist` | double | `3.0` | 允许开火的最大距离（m） |
| `yaw_tolerance_deg` | double | `5.0` | 允许开火的 yaw 误差（度） |
| `pitch_tolerance_deg` | double | `2.0` | 允许开火的 pitch 误差（度） |
| `min_spinning_vel` | double | `5.0` | 进入小陀螺判定的角速度门限（rad/s） |
| `spin_enter_time` | double | `0.3` | 持续多久判定为小陀螺（s） |
| `spin_exit_time` | double | `0.15` | 低于门限多久退出小陀螺（s） |

> 表格里的"代码默认值"只是兜底。实际值以 launch 传入的 YAML 为准，例如通用 `tracker_params.yaml` 里 `bullet_speed: 32.0`、`shootable_dist: 10.0`、`min_spinning_vel: 1.0`、`ekf_QR_r_xyz_factor: 0.5`，与代码默认值差别很大。

运行时（`add_on_set_parameters_callback`）只有 3 个参数支持热改：`target_color`、`bullet_speed`（会重建弹道表）、`display`。其余参数改了不生效。

## 使用方式

### 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/DT46-V
colcon build --packages-select rm_interfaces rm_tracker --symlink-install
source install/setup.bash
```

### launch 启动

```bash
ros2 launch rm_tracker tracker.launch.xml
# 换参数文件
ros2 launch rm_tracker tracker.launch.xml tracker_params_file:=/abs/path/tracker_params_sentry.yaml
```

launch argument：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `tracker_params_file` | `share/rm_tracker/config/tracker_params.yaml` | 参数文件路径 |

### ros2 run

```bash
ros2 run rm_tracker rm_tracker_node --ros-args \
  --params-file install/rm_tracker/share/rm_tracker/config/tracker_params_infantry.yaml
```

### 动态调参

```bash
ros2 param set /rm_tracker target_color 1     # 切蓝方，会重置状态机
ros2 param set /rm_tracker bullet_speed 30.0  # 重建弹道表
ros2 param set /rm_tracker display true       # 打开调试图
```

### 整机启动 / 单独验证

- 整机：`ros2 launch rm_vision_bringup hik.launch.xml`（哨兵）、`mv.launch.xml`（步兵）、`hero.launch.xml`（英雄）、`sentry.launch.xml`。
- 纯视觉（无串口）：`hik_nav.launch.xml` / `mv_nav.launch.xml` / `usb_nav.launch.xml`；注意这些场景下 `/nav/decision` 不存在，`follow_decision` 必须为 false 才能按 YAML 里的颜色打。
- 手动验证链路：`ros2 topic echo /detector/armors_info` → `ros2 topic echo /tracker/gimbal_control`。

## 依赖与前置条件

- ROS 包依赖：`rclcpp`、`sensor_msgs`、`geometry_msgs`、`std_msgs`、`cv_bridge`、`rm_interfaces`、`OpenCV`、`Eigen3`。
- 编译选项：`-Wall -Wextra -Wpedantic -O3 -march=native`。
- 运行前置：
  - `/detector/armors_info`（`rm_detector`）
  - `/imu/rpy`（`dm_imu`，或串口节点 `pub_rpy: true`；单位必须是度）
  - `/camera_info`（相机驱动；只用于渲染与投影，不参与 EKF）

## 注意事项 / 已知问题

1. **YAML 里有一批"写了但不生效"的参数**：`log_throttle_ms`（代码里硬编码 1000 ms）、`show_rpy`、`debug`、`max_spinning_frame`、`spinning_frame_lost`、`distance_decress_ratio` 在 `node.cpp` 里没有 `declare_parameter`，属于历史遗留（早期帧计数版小陀螺），配置里留着会让人误以为可以调。当前小陀螺是**按时间**累计的（`spin_enter_time` / `spin_exit_time`）。
2. **不声明就改不动**：ROS 2 对未声明的参数只会当作 override 忽略，不会有任何报错或告警，排查参数问题时建议先 `ros2 param list /rm_tracker` 核对。
3. **颜色过滤只在节点层做**：`Tracker::target_color` 成员在 `tracker.cpp` 里没有被使用，实际过滤发生在 `node.cpp` 的 `armors_cb`。
4. **渲染依赖 `/camera_info`**：`res_img_cb` 在 `tf_.has_camera_info()` 为 false 时直接返回，表现为"开了 display 也没有图像"。
5. **`/nav/decision` 缺失时不会切色**：`follow_decision: true` 的英雄/步兵配置依赖 `rm_serial` 转发裁判系统颜色；纯视觉 launch 下该话题不存在，会一直用 YAML 里的 `target_color`。
6. **两个 `/imu/rpy` 发布者会冲突**：`dm_imu`（默认开）与 `rm_serial`（`pub_rpy`，默认 false）都会发这个话题，且 `rm_serial` 直通下位机数据、单位不一定与 `dm_imu` 一致，不要同时开。
7. **`dt` 保护会掩盖时间戳问题**：帧间隔异常（≤1ms 或 >30ms）会被强制成 10ms，如果发现速度和预测明显异常，先确认 `/detector/armors_info` 的时间戳是否正常。
