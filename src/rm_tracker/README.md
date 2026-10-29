# rm_tracker

装甲板追踪与弹道解算节点（C++ / ROS 2 Humble）。消费 `rm_detector` 的装甲板观测、`dm_imu` 的姿态和相机内参，完成

相机系 → 世界系坐标变换 → 目标选择/装甲板跳变处理 → 11 维共享车心 EKF 状态估计 → 反小陀螺判定 → 空气阻力弹道解算 → 输出云台 `yaw/pitch` 与 `can_fire`。

代码分为两层：`tracker_core`（`tracker.cpp` / `body_ekf.cpp` / `rm_tf_tools.cpp`，纯算法；`ekf.cpp` 保留作旧算法辅助）与 `rm_tracker_node`（`node.cpp`，ROS 通信、线程、渲染）。

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
- 发出的 yaw/pitch 就是发弹门控所评估的原始解，不再于门控之后额外 EMA 平滑。

## 数据流与算法

当前实验分支 `codex/relative-aim-restart` 重写车体更新与预测；下面描述当前代码，不再保证与 Glenn tracker 源码一致。当前 HOLD、切板、GUI 与闭环验证见[自瞄链路复核](../../outputs/aim_model_validation_20261029/report.md)；先前参考来源与同输入基线比较见[底层修复实验](../../outputs/body_model_repair_20261029/report.md)。仿真与理想传感器实验不能据此宣称实车命中。

1. **输入与坐标**：颜色过滤和相机坐标变换沿用现有链路。只用独立视觉 IMU，不补 MCU 姿态；变换后的坐标仍是以移动相机为原点的相对目标位置，不能将速度当成敌方绝对速度。ROS 消息已有相机系板法线 `nx/ny/nz`；ROS 和原生入口现在都先旋转完整法线，再由水平投影取得板→心朝向，避免仅相减 yaw 丢失相机俯仰关系。法线无效时回退旧 yaw。本轮未新增消息或硬件能力。
2. **时间**：ROS 和原生入口共用 `FrameClock`。首帧建轨不预测；重复、倒序、非有限戳拒绝且不移动时间基准；所有正间隔原样使用，包括 0.5 ms 和 200 ms。超过 0.5 s 断流失效、重建轨，不把长间隔截成短间隔。这个阈值是当前恢复策略，不是已测延迟。主机戳的采样误差依然存在，配置补偿语义见[独立传感器实验](../../outputs/relative_aim_restart_20261029/report.md#延迟配置与计算语义)。
3. **共享车体**：`BodyEkf` 状态为 `[cx,vx,cy,vy,zA,vz,theta,omega,rA,rB,hB]`。四板角度为 `theta+i*pi/2`，位置为 `(cx-r*cos(angle),cy-r*sin(angle),zA+group*hB)`，组别取 `i%2`。同组相差 180°且保留同组高度；两个半径和高度差独立估计。zA 是 A 组板平面，不是已测底盘高度。90°是名义拓扑，车型表半径仅作先验。初始化由 `center=board+radius*normal` 的雅可比传播观测与几何先验，保留车心、半径、相位的交叉协方差；单块静止板不能唯一确定真实车心。
4. **关联与融合**：匿名观测相对于同一预测状态枚举四板，使用 PnP 已选定并经完整坐标变换的原始 yaw，仅将角差绕回 `[-pi,pi]`。不再按预测方向人为反射或 pi 翻转观测，也不再维护由这种改写支撑的初始镜像车体；否则正确 PnP 能被改成错误测量，并以接近零的创新锁住错误朝向。NIS 上限 18.47，候选分差须大于 1；重复分配只更新一次，同帧最多处理 16 个候选、更新四板。没有半径交换、换板冷却或冻结车心。连续白噪声 Q 使用真实 dt，LDLT 检查正定性，Joseph 更新并对称化 P。位置 R 支持沿视线增加随距离平方增长的深度方差；关闭该项时恢复各向同性 R。修复前后同日志对照及仍存在的原始 PnP 偏差见[静止画面复核](../../outputs/pnp_static_diagnosis_20261029/report.md)。
5. **状态机**：建轨/失锁按真实累计时间；`tracking_thres`/`lost_thres` 按名义 10 ms 换算持续时间。建轨中一次漏配会清零连续确认时间；只有车体关联成功才算有效跟踪。无效时间清除本帧瞄准与匹配标记，超过断流阈值重建。SPIN 进入/退出使用共享车体角速度及配置的持续时间，不再由旧单板跳变决定发射窗口。
6. **选板与预测**：共享车心按相对速度、相位按角速度传播，枚举到达时刻真实四板；选板有 0.08 rad 切换惩罚，不交换 EKF 的半径或高度组。选板和瞄准共用 `飞行时间+system_delay+controller_delay`，两个补偿项均以秒配置、不考虑则写 0，当前配置不是实测延迟。飞行时间以车体近表面距离作一次初估，弹道解算后重检到达位置，偏差超过 25 mm 拒绝，没有迭代求解器。
7. **弹道与许可**：沿用平方阻力 LUT 和固定重力 9.81；0.2 m 起查表，表外拒绝。必须处于 TRACKING、本帧车体匹配、至少接受 8 次更新、弹道有效且预测一致。将包含未来 Q 的 P 投影到选中板切向和竖直方向，两倍标准差分别不得超过半板宽的 85% 和半板高的 80%；不再强制未观测的另一组半径收敛。`body_ready` 仍表示完整几何协方差条件，仅作诊断。角度许可同时受板面投影、配置上限及 SPIN 正对弧长约束；没有远距离 1° 下限，也不再让 HUD 覆盖配置。输出不再门控后平滑。这些是高斯模型下的工程门限，不能保证所有模型偏差下的命中。

旧单板估计与九维 EKF、圆拟合/冻结车心、半径交换辅助函数保留供兼容调试和对照，已退出车体更新、预测与 SPIN 发射判定。原生 GUI 的 `tracking_state_img` 显示估计车心、四块名义竖直模型板、匹配/选中标记；`ballistic_img` 显示 HOLD 原因和到达标准差，默认四视图包括这两路。板框按车型表名义宽高绘制，不是完整倾斜 CAD 投影。原生适配器仍有光学/枪口真值输入；ROS 及实车尚未验证，不能将原生闭环称为实车输入公平的证据。
## 参数

参数文件：`config/tracker_params.yaml`（通用）、`tracker_params_hero.yaml`、`tracker_params_infantry.yaml`、`tracker_params_sentry.yaml`。

| 参数 | 类型 | ROS declare 默认值 | 含义 |
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
| `max_match_yaw_diff` | double | `57.0` | 车体观测关联的 yaw 偏差上限（度） |
| `jump_cooldown_max` | int | `20` | 保留兼容参数；当前车体关联不使用跳变冷却 |
| `tracking_thres` | int | `5` | 有效关联持续时间 / 10 ms |
| `lost_thres` | int | `20` | 丢失持续时间 / 10 ms |
| `dist_tol` | double | `0.15` | 保留兼容；当前建轨按三维距离选最近板 |
| `ekf_QR_q_xyz` | double | `0.2` | 相对平移加速度过程噪声谱密度（m²/s³） |
| `ekf_QR_q_yaw` | double | `15.0` | 角度过程噪声谱密度 |
| `ekf_QR_q_r` | double | `0.000001` | 两组半径及高度差的游走谱密度（m²/s） |
| `ekf_QR_r_xyz_factor` | double | `0.05` | 基础位置 sigma（m）：`0.005+0.01*factor*dist`，dist 以米计 |
| `ekf_QR_r_depth_factor` | double | `0.0` | 0 关闭额外深度噪声；正值时深度 sigma 为 `max(基础sigma,0.005+0.01*factor*dist²)`，只增加视线方向方差 |
| `ekf_QR_r_yaw` | double | `0.02` | 角度观测噪声方差 |
| `ekf_QR_stable_dist` | double | `1.5` | 动态调整 Q/R 的参考距离（m） |
| `radius_r_max` / `radius_r_min` | double | `0.4` / `0.12` | 车辆半径限幅（m） |
| `system_delay` | double | `0.1` | 系统延迟补偿（s） |
| `controller_delay` | double | `0.02` | 额外执行补偿（s）；两项均支持 0，不能据配置值宣称测得延迟 |
| `k_v2` | double | `0.019` | 平方空气阻力系数 |
| `shootable_dist` | double | `3.0` | 允许开火的最大距离（m） |
| `yaw_tolerance_deg` | double | `5.0` | 允许开火的 yaw 误差（度） |
| `pitch_tolerance_deg` | double | `2.0` | 允许开火的 pitch 误差（度） |
| `min_spinning_vel` | double | `5.0` | 进入小陀螺判定的角速度门限（rad/s） |
| `spin_enter_time` | double | `0.3` | 持续多久判定为小陀螺（s） |
| `spin_exit_time` | double | `0.15` | 低于门限多久退出小陀螺（s） |

> 表格里的"代码默认值"只是兜底。实际值以 launch 传入的 YAML 为准，例如通用 `tracker_params.yaml` 里 `bullet_speed: 25.0`、`shootable_dist: 10.0`、`min_spinning_vel: 1.0`、`ekf_QR_r_xyz_factor: 0.5`，与代码默认值差别很大。

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
ros2 param set /rm_tracker bullet_speed 25.0  # 重建弹道表
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
  - `/imu/rpy`（当前分支使用独立 `dm_imu`；单位必须是度，不接 MCU 姿态）
  - `/camera_info`（相机驱动；只用于渲染与投影，不参与 EKF）

## 注意事项 / 已知问题

1. **YAML 里有一批"写了但不生效"的参数**：`log_throttle_ms`（代码里硬编码 1000 ms）、`show_rpy`、`debug`、`max_spinning_frame`、`spinning_frame_lost`、`distance_decress_ratio` 在 `node.cpp` 里没有 `declare_parameter`，属于历史遗留（早期帧计数版小陀螺），配置里留着会让人误以为可以调。当前进入 SPIN 依赖连续同向换板，退出结合时间门限；不能把 `min_spinning_vel` / `spin_enter_time` 理解为唯一进入条件。
2. **不声明就改不动**：ROS 2 对未声明的参数只会当作 override 忽略，不会有任何报错或告警，排查参数问题时建议先 `ros2 param list /rm_tracker` 核对。
3. **颜色过滤在核心外做**：`Tracker::target_color` 成员在 `tracker.cpp` 里没有被使用；ROS 在 `node.cpp` 的 `armors_cb` 过滤，原生接入在适配层执行相同过滤。
4. **渲染依赖 `/camera_info`**：`res_img_cb` 在 `tf_.has_camera_info()` 为 false 时直接返回，表现为"开了 display 也没有图像"。
5. **`/nav/decision` 缺失时不会切色**：`follow_decision: true` 的英雄/步兵配置依赖 `rm_serial` 转发裁判系统颜色；纯视觉 launch 下该话题不存在，会一直用 YAML 里的 `target_color`。
6. **两个 `/imu/rpy` 发布者会冲突**：`dm_imu`（默认开）与 `rm_serial`（`pub_rpy`，默认 false）都会发这个话题，且 `rm_serial` 直通下位机数据、单位不一定与 `dm_imu` 一致，不要同时开。
7. **时间戳仍需测量**：本轮已移除 dt 替换/截断，但相机与 IMU 主机戳不等于曝光/硬件采样时间；真实同步和抖动仍未验证。
