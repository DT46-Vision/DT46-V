# rm_tracker

装甲板追踪与弹道解算节点（C++ / ROS 2 Humble）。消费 `rm_detector` 的装甲板观测、`dm_imu` 的姿态和相机内参，完成

相机系 → 世界系坐标变换 → 目标选择/装甲板跳变处理 → 11 维共享车心 EKF 状态估计 → 反小陀螺判定 → 空气阻力弹道解算 → 输出云台 `yaw/pitch` 与 `can_fire`。

代码分为两层：`tracker_core`（`tracker.cpp` / `body_ekf.cpp` / `rm_tf_tools.cpp`，纯算法；`ekf.cpp` 仅留作历史回归，当前 Tracker 不使用旧九维滤波器）与 `rm_tracker_node`（`node.cpp`，ROS 通信、线程、渲染）。

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

当前实验分支使用共享车体更新与预测；下面描述当前代码，不再保证与 Glenn tracker 源码一致。当前 HOLD、切板、GUI 与闭环验证见[自瞄链路复核](../../outputs/aim_model_validation_20261029/report.md)；先前参考来源与同输入基线比较见[底层修复实验](../../outputs/body_model_repair_20261029/report.md)。仿真与理想传感器实验不能据此宣称实车命中。

1. **输入与坐标**：颜色过滤和相机坐标变换沿用现有链路。只用独立视觉 IMU，不补 MCU 姿态；变换后的坐标仍是以移动相机为原点的相对目标位置，不能将速度当成敌方绝对速度。ROS 消息已有相机系板法线 `nx/ny/nz`；ROS 和原生入口现在都先旋转完整法线，再由水平投影取得板→心朝向，避免仅相减 yaw 丢失相机俯仰关系。法线无效时回退旧 yaw。本轮未新增消息或硬件能力。
2. **时间**：ROS 和原生入口共用 `FrameClock`。首帧建轨不预测；重复、倒序、非有限戳拒绝且不移动时间基准；所有正间隔原样使用，包括 0.5 ms 和 200 ms。超过 0.5 s 断流失效、重建轨，不把长间隔截成短间隔。这个阈值是当前恢复策略，不是已测延迟。主机戳的采样误差依然存在，配置补偿语义见[独立传感器实验](../../outputs/relative_aim_restart_20261029/report.md#延迟配置与计算语义)。
3. **共享车体**：`BodyEkf` 状态为 `[cx,vx,cy,vy,zA,vz,theta,omega,rA,rB,hB]`。四板角度为 `theta+i*pi/2`，位置为 `(cx-r*cos(angle),cy-r*sin(angle),zA+group*hB)`，组别取 `i%2`。同组相差 180°且保留同组高度；两个半径和高度差独立估计。zA 是 A 组板平面，不是已测底盘高度。90°是名义拓扑；两组半径均以配置上下限的中点初始化，高差初值为 0，不按车辆编号填半径。半径受配置上下限保护，高差绝对值上限为 0.3 m；这些是模型适用范围，不是测得几何。初始化由 `center=board+radius*normal` 的雅可比传播观测与几何先验，保留车心、半径、相位的交叉协方差；单块静止板不能唯一确定真实车心。
4. **关联与融合**：匿名观测相对于同一预测状态枚举四板，使用 PnP 已选定并经完整坐标变换的原始 yaw，仅将角差绕回 `[-pi,pi]`。不再按预测方向人为反射或 pi 翻转观测，也不再维护由这种改写支撑的初始镜像车体；否则正确 PnP 能被改成错误测量，并以接近零的创新锁住错误朝向。NIS 上限 18.47，候选分差须大于 1；重复分配只更新一次，同帧最多处理 16 个候选、更新四板。没有半径交换、换板冷却或冻结车心。连续白噪声 Q 使用真实 dt，LDLT 检查正定性，Joseph 更新并对称化 P。位置 R 支持沿视线增加随距离平方增长的深度方差；启用时硬关联分别检查横向 `max_match_distance` 和深度 `max(max_match_distance,3*depth_sigma)`，之后仍须通过 NIS；关闭该项时保持原三维距离门槛及各向同性 R。修复前后同日志对照及仍存在的原始 PnP 偏差见[静止画面复核](../../outputs/pnp_static_diagnosis_20261029/report.md)。
5. **状态机**：建轨/失锁按真实累计时间；`tracking_thres`/`lost_thres` 按名义 10 ms 换算持续时间。建轨中一次漏配会清零连续确认时间；只有车体关联成功才算有效跟踪。无效时间清除本帧瞄准与匹配标记，超过断流阈值重建。SPIN 进入/退出使用共享车体角速度及配置的持续时间，不再由旧单板跳变决定发射窗口。
6. **选板与预测**：共享车心按相对速度、相位按角速度传播，枚举到达时刻真实四板；选板有 0.08 rad 切换惩罚，不交换 EKF 的半径或高度组。选板和瞄准共用 `飞行时间+system_delay+controller_delay`，两个补偿项均以秒配置、不考虑则写 0，当前配置不是实测延迟。飞行时间以车体近表面距离作一次初估，弹道解算后重检到达位置，偏差超过 25 mm 拒绝，没有迭代求解器。
7. **弹道与发弹**：弹道使用下述直接解算；必须处于 TRACKING、本帧车体匹配、至少接受 8 次更新、弹道有效且预测一致。将包含未来 Q 的 P 投影到视线横向和竖直方向，用当前枪口误差作为均值，计算落入投影板面宽 85%、高 80% 区域的两个一维高斯概率之积；须达到 `min_hit_probability`。步兵当前取 0.25，其余配置为 0.5。还须满足实际板面角窗、配置误差上限、正面可见和射程要求。删除独立 SPIN 正对弧长限制及“两倍 sigma 必须塞进半板”的重复门槛。该概率忽略相关项、模型偏差和弹丸散布，不是实测命中率。`body_ready` 仅作完整几何收敛诊断，未观测的另一组半径不阻止已观测板发射。

当前运行路径只有共享车体模型；旧单板滤波、圆拟合、冻结车心、换板周期及半径交换辅助代码已移除。适配器报告的板位置/速度也从同一车体的当前观测板求出，包含切向速度。原生 GUI 的 `tracking_state_img` 显示估计车心、四块名义竖直模型板、匹配/选中标记；`ballistic_img` 显示 HOLD 原因和到达标准差。JSON 另含 `aim_hit_probability`。板框使用名义 135×125 mm，不是完整倾斜 CAD 投影。算法处理同一目标的多块板；当前仍只锁定一个 `tracked_id`，没有多车独立轨迹池，同编号车辆的身份分离尚未实现。

仿真闭环、固定日志回放、半径/正负高差回归和当前边界见[视觉追踪优化记录](../../outputs/vision_tracking_20261030/report.md)。原生适配器继续使用已有光学/枪口姿态输入；新算法未增加目标真值输入。ROS 与实车属于后续阶段。

步兵平移噪声按[5 m 逐发诊断](../../outputs/vision_accuracy_20261030/report.md)调整：基础横向/竖直观测噪声与测深噪声分开，改善速度变化后的跟随；具体值只维护在 `tracker_params_infantry.yaml`。没有增加状态维数、在线求解器或放宽发弹门控。匀速模型在急刹反向期间仍有明显预测偏差；上述概率不能覆盖这种模型误差。静止远距离命中还受发射散布限制，单凭命中比例不能归因于 PnP 或弹道公式。

### 直接弹道解算

`ballistics.hpp::solve_projectile` 一次返回低弹道俯仰角、飞行时间和有效性。使用固定重力 9.81 m/s²、当前弹速和 `k_v2`，先求无阻力低弹道，再由其四分之一路程的斜率估算有效阻力，直接解二次式；没有在线试射、表格生成或收敛循环。原 0.5 m 网格、五轮试射制表及角度/飞行时间两次插值已删除。飞行时间与选板、误差预测复用同一次解算结果；适配器读取本帧有效性，不再重算弹道。

保留水平距离 0.2–12 m、高差 ±3.5 m 的范围保护，非有限值、非正弹速、负阻力、无低弧解拒绝。后方或超过 ±1 rad 指令范围的目标输出限幅转向值，但有效性为 false。精度验证仅覆盖已执行的有限网格：20–30 m/s、`k_v2=0.019`（另测零阻力）；其他参数仍可配置，尚无同等误差界。模型不包含风、Magnus 或非零线性阻力。

公式误差、MuJoCo 3.13.0 对照、相同 RGB 输入测速与仿真发弹结果见[视觉速度与弹道验证](../../outputs/vision_speed_20261030/report.md)。MuJoCo 只用于离线对照，运行时不新增依赖。

## 参数

参数文件：`config/tracker_params.yaml`（通用）、`tracker_params_hero.yaml`、`tracker_params_infantry.yaml`、`tracker_params_sentry.yaml`。

| 参数 | 类型 | ROS declare 默认值 | 含义 |
| --- | --- | --- | --- |
| `target_color` | int | `0` | 要打的颜色：0 = 红（保留 id ≥ 6），1 = 蓝（保留 id < 6） |
| `follow_decision` | bool | `false` | true 时用 `/nav/decision` 的 `color` 覆盖 `target_color`（覆盖会重置状态机为 LOST） |
| `bullet_speed` | double | `28.0` | 弹速（m/s）；`/nav/decision` 的 `bullet_speed > 10` 时也会自动同步；不再重建弹道表 |
| `display` | bool | `false` | 是否发布两路调试图 |
| `display_fps_limit` | bool | `true` | 渲染限速 30 FPS |
| `text_size` | double | `1.0` | 调试图文字缩放 |
| `rotation_rpy_r/p/y` | double | `0 / 0 / −180` | IMU → 相机的轴固定旋转修正（度） |
| `cam_to_gun_pos_x/y/z` | double | `0 / 0.125 / 0` | 相机 → 枪口平移外参（m） |
| `cam_to_gun_rpy_r/p/y` | double | `0 / 0 / 0` | 相机 → 枪口旋转外参（度） |
| `max_match_distance` | double | `0.2` | 观测关联位置门槛（m）；额外深度噪声启用时按上面的横向/深度规则处理 |
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
| `min_hit_probability` | double | `0.5` | 到达板面的高斯近似概率下限，范围 (0,1]；步兵配置 0.25 |
| `min_spinning_vel` | double | `5.0` | 进入小陀螺判定的角速度门限（rad/s） |
| `spin_enter_time` | double | `0.3` | 持续多久判定为小陀螺（s） |
| `spin_exit_time` | double | `0.15` | 低于门限多久退出小陀螺（s） |

> 表格里的"代码默认值"只是兜底。实际值以 launch 传入的 YAML 为准，例如通用 `tracker_params.yaml` 里 `bullet_speed: 25.0`、`shootable_dist: 10.0`、`min_spinning_vel: 1.0`、`ekf_QR_r_xyz_factor: 0.5`，与代码默认值差别很大。

运行时（`add_on_set_parameters_callback`）只有 3 个参数支持热改：`target_color`、`bullet_speed`（直接供下一帧解算使用）、`display`。其余参数改了不生效。

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
ros2 param set /rm_tracker bullet_speed 25.0  # 下一帧直接弹道解算使用新弹速
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

1. **YAML 里有一批"写了但不生效"的参数**：`log_throttle_ms`（代码里硬编码 1000 ms）、`show_rpy`、`debug`、`max_spinning_frame`、`spinning_frame_lost`、`distance_decress_ratio` 在 `node.cpp` 里没有 `declare_parameter`，属于历史遗留（早期帧计数版小陀螺），配置里留着会让人误以为可以调。当前 SPIN 按共享车体角速度和持续时间判定，见上面的状态机说明。
2. **不声明就改不动**：ROS 2 对未声明的参数只会当作 override 忽略，不会有任何报错或告警，排查参数问题时建议先 `ros2 param list /rm_tracker` 核对。
3. **颜色过滤在核心外做**：`Tracker::target_color` 成员在 `tracker.cpp` 里没有被使用；ROS 在 `node.cpp` 的 `armors_cb` 过滤，原生接入在适配层执行相同过滤。
4. **渲染依赖 `/camera_info`**：`res_img_cb` 在 `tf_.has_camera_info()` 为 false 时直接返回，表现为"开了 display 也没有图像"。
5. **`/nav/decision` 缺失时不会切色**：`follow_decision: true` 的英雄/步兵配置依赖 `rm_serial` 转发裁判系统颜色；纯视觉 launch 下该话题不存在，会一直用 YAML 里的 `target_color`。
6. **两个 `/imu/rpy` 发布者会冲突**：`dm_imu`（默认开）与 `rm_serial`（`pub_rpy`，默认 false）都会发这个话题，且 `rm_serial` 直通下位机数据、单位不一定与 `dm_imu` 一致，不要同时开。
7. **时间戳仍需测量**：本轮已移除 dt 替换/截断，但相机与 IMU 主机戳不等于曝光/硬件采样时间；真实同步和抖动仍未验证。
