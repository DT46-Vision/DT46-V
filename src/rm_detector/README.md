# rm_detector

算法核可由 [standalone](../../standalone/README.md) 无 ROS 构建；`PNP` 接收普通 `CameraCalibration`，ROS 节点负责把 `CameraInfo` 转换为该结构，话题接口不变。

装甲板检测与三维解算节点（C++ / ROS 2 Humble）。输入相机图像与内参，输出每帧装甲板在**相机系**下的坐标 `(dx, dy, dz)`（单位 mm）与相对偏航角 `yaw`（单位度）。

灯条提取 → 双灯条配对 → 透视校正 + Otsu 二值化 → CNN 数字分类 → 颜色/类别映射出 `armor_id` → 板位姿解算 解算位姿。

## 节点与可执行文件

| 节点名（ROS 图中的名字） | 可执行文件 | 作用 |
| --- | --- | --- |
| `rm_detector` | `rm_detector_node` | 装甲板检测 + 测距 + yaw 解算，发布装甲板消息与调试图像 |

- 节点名硬编码在 `ArmorDetectorNode() : Node("rm_detector")`（`src/node.cpp`）。
- 因此参数 YAML 的根键必须写 `rm_detector:`，否则参数不会生效（`config/` 下 4 个文件都符合）。
- 单进程单线程模型：构造函数里另起一个 `processing_loop` 工作线程做识别，`image_callback` 只负责存"最新一帧"并唤醒它。

## 话题接口

| 方向 | 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- | --- |
| 订阅 | `/image_raw` | `sensor_msgs/msg/Image` | SensorDataQoS（BestEffort） | 输入图像，需能转成 BGR8 |
| 订阅 | `/camera_info` | `sensor_msgs/msg/CameraInfo` | SensorDataQoS | 相机内参，仅第一帧被解析并缓存 |
| 发布 | `/detector/armors_info` | `rm_interfaces/msg/ArmorsMsg` | SensorDataQoS | 主输出：每块装甲板的 `armor_id, dx, dy, dz, yaw` |
| 发布 | `/detector/armors_debug_info` | `rm_interfaces/msg/ArmorsDebugMsg` | SensorDataQoS | 调试：四角灯条像素坐标 + 颜色 + id |
| 发布 | `/detector/result` | `sensor_msgs/msg/Image` | SensorDataQoS | 调试图像，**仅 `display_mode: true` 时发布** |
| 发布 | `/detector/crop_img` | `sensor_msgs/msg/Image` | SensorDataQoS | 同上，ROI 裁剪图（`roi_crop` 关闭时是 1x1 占位） |
| 发布 | `/detector/bin_img` | `sensor_msgs/msg/Image` | SensorDataQoS | 同上，二值图（mono8） |
| 发布 | `/detector/img_armor` | `sensor_msgs/msg/Image` | SensorDataQoS | 同上，透视校正后的装甲板图 |
| 发布 | `/detector/img_armor_processed` | `sensor_msgs/msg/Image` | SensorDataQoS | 同上，送进 CNN 的二值图（mono8） |
| 发布 | `/detector/img_debug` | `sensor_msgs/msg/Image` | SensorDataQoS | 同上，带框标注的整图 |

- 所有消息的 `header.stamp` 直接沿用输入图像的时间戳（不重新取时钟），所以整条链路时间对齐，bag 回放时不需要 `--clock`。
- `header.frame_id` 固定为 `camera_frame`。
- 无论本帧是否检测到装甲板，`/detector/armors_info` 与 `/detector/armors_debug_info` 都会发布（空数组），便于追踪节点判断"丢失"。

## 处理流程

1. `process()`：默认接受 BGR；原生入口显式指定 RGB，灰度化和红蓝采样直接按输入通道读取，省去整帧 RGB→BGR。灰度 → 固定阈值二值化（`binary_val`）；`roi_crop: true` 时先按 `roi_scale` 取图像中心区域，后续只在 ROI 上处理。
2. `find_lights()`：轮廓面积 ≥ `light_area_min`、minAreaRect 的 高/宽 ≥ `light_h_w_ratio`、角度在 `[light_angle_min, light_angle_max]` 内；再沿灯条中心线采样最多 10 个像素做红蓝判决。默认配置要求红蓝强度比超过 1.3，排除接近无色的白字。端点沿中心线在 ±2 px 内固定采样局部灰度剖面，线性插值半对比度边界，避免直接把轮廓像素中心当作物理端点；对比度不足或贴边时保留原端点。
3. `is_armor()`：按中心 x 排序，依次检查右侧同色候选的灯条夹角、中心连线角、高度比 < `height_rate_tol`、中心距/平均高严格位于 `(height_multiplier_min, height_multiplier_max)` 内，再进行分类；几何或分类失败就继续检查后续右灯。接受首个完整通过的配对后，占用两根灯条和实际二维板面；候选板面有面积重叠时拒绝，横向区间重叠但上下分离的板可以同时输出。灯条夹角仍按最小角差计算，避免近竖直灯条跨越 ±180° 时被误拒。
4. `get_armor_result()`：透视变换到 89×125 → Otsu 二值化 → `NumberClassifier` 推理 → `class_id`。
5. `Armor::get_id()`：颜色 + `class_id` 映射成 `armor_id`（见下表）；`is_armor()` 只接受 `class_id ∈ {0,1,2}`。
6. `PNP::processArmorCorners()`：未设置姿态约束时沿用 IPPE 双解并按当前四角重投影均方误差选择。原生步兵配置 `pnp_armor_tilt_deg: 15`，复用相机光学旋转，将板的两个面内方向写成 yaw 正余弦的线性式；用去畸变后的四条射线解 8×5 线性系统，归一化 yaw，再解一次 8×3 平移系统。没有迭代搜索、历史法线覆盖或目标真值输入；背面、负深度、不合理角向量及 RMS > 2 px 的受约束解被拒绝。安装倾角假设不适用的目标应移除此配置，不能用它代替真实姿态测量。输出仍为 `dx/dy/dz`（mm）、`yaw`（deg）与相机系向外单位法线；灯距/灯高由配置给出，与车辆编号、车体半径和对角组高差无关。ROS 检测节点尚未接入该相机姿态约束，仍使用 IPPE。

原生跟踪图把检测端点（黄色）、原始 PnP 重投影与 120 mm 法线箭头（品红）和 EKF 模型（青/绿/橙）分开显示；PnP 行显示深度和像素 RMS。重投影贴合只说明位姿解释了输入角点，不能证明尺寸、角点与真实物体一致。完整平面法线在近正视或小像素目标上仍可能不可靠，本次静止及旋转的离线检查见[底层 PnP 复核](../../outputs/pnp_scene_cleanup_20261029/report.md)。

每轮配对清空上一轮装甲结果与分类预览；空输入清空灯条与装甲缓存。保存帧回归覆盖白字、灰度假灯、相邻板、上下重叠区间、交叉反例、亚像素边界、ROI 和缓存清理。当前实验与剩余精度边界见[视觉追踪优化记录](../../outputs/vision_tracking_20261030/report.md)；前一阶段证据保留在[检测器精简验证](../../outputs/detector_simplify_20261030/report.md)。部分可见灯条仍只是候选，不虚构缺失端点或强制拼成完整装甲；多车独立跟踪由 tracker 的能力边界决定。

## 参数

`config/` 下有 4 份参数文件；步兵文件另显式列出 PnP 灯条尺寸，未填写时使用代码默认值：
`detector_params.yaml`（通用，0.6）、`detector_params_sentry.yaml`（0.7）、`detector_params_hero.yaml`（0.7）、`detector_params_infantry.yaml`（0.7）。

| 参数 | 类型 | 代码 declare 默认值 | config 中的值 | 含义 |
| --- | --- | --- | --- | --- |
| `cls_model_file` | string | `mlp.onnx` | `RM_Armor_CNN_sim.onnx` | ONNX 文件名（相对 `share/rm_detector/model/`）；置空字符串可禁用分类器 |
| `pnp_light_spacing_mm` | double | `135.0` | 步兵 `135.0`，其余使用默认 | 两灯条中心距，必须为正有限数；不是装甲板整体宽度 |
| `pnp_light_height_mm` | double | `58.0` | 步兵 `58.0`，其余使用默认 | 与检测端点对应的有效灯条长度，必须为正有限数；实际硬件须标定 |
| `pnp_armor_tilt_deg` | double | 未设置 | 原生步兵 `15.0` | 板外法线仰角，范围 (-60,60)；配置存在且有相机姿态时使用直接约束解 |
| `roi_crop` | bool | `false` | `false` | 是否只处理图像中心 ROI |
| `roi_scale` | double | `0.5` | `0.4` | ROI 占整图的比例 |
| `light_area_min` | int | `5` | `5` | 灯条轮廓最小面积（像素） |
| `light_h_w_ratio` | double | `5.0` | `2.0` | 灯条高/宽比下限，小于它直接丢弃 |
| `light_angle_min` | int | `-35` | `-35` | 灯条倾角下限（度） |
| `light_angle_max` | int | `35` | `35` | 灯条倾角上限（度） |
| `light_red_ratio` | double | `2.0` | `1.3` | 红判定阈值：`sum_r > sum_b * light_red_ratio` |
| `light_blue_ratio` | double | `2.0` | `1.3` | 蓝判定阈值：`sum_b > sum_r * light_blue_ratio` |
| `height_rate_tol` | double | `1.3` | `1.8` | 两灯条高度比上限 |
| `height_multiplier_min` | double | `1.8` | `1.0` | 中心距/平均灯条高 的下限 |
| `height_multiplier_max` | double | `3.0` | `3.0` | 中心距/平均灯条高 的上限 |
| `row_stretch` | double | `2.22` | `2.22` | 灯条端点纵向拉伸系数（装甲板匹配用） |
| `col_stretch` | double | `0.66` | 0.6 / 0.7 | 灯条端点横向拉伸系数 |
| `binary_val` | int | `120` | `120` | 固定二值化阈值 |
| `detect_color` | int | `2` | `2` | 0=只找红灯条、1=只找蓝灯条、2=红蓝都找 |
| `display_mode` | bool | `false` | `false` | 是否发布 6 路调试图像 |
| `use_geometric_center` | bool | `true` | `true` | PnP 时用图像几何中心覆盖内参 `cx/cy` |
| `print_period_ms` | int | `1000` | `1000` | 终端检测/FPS 打印周期（ms），≤0 表示每帧都打 |

> 代码里 declare 的默认值只是兜底；实际运行时由 launch 传入的 YAML 覆盖，**以 YAML 为准**。

运行时可以动态生效的参数（有 `add_on_set_parameters_callback` 处理）：
`cls_model_file`（重新加载模型）、`light_area_min`、`light_h_w_ratio`、`light_angle_min/max`、`light_red_ratio`、`light_blue_ratio`、`height_rate_tol`、`height_multiplier_min/max`、`row_stretch`、`col_stretch`、`binary_val`、`detect_color`、`display_mode`、`print_period_ms`、`roi_crop`、`roi_scale`。
**`use_geometric_center` 不在回调里**，运行时改它会被接受但不生效。

## armor_id 映射（重要）

装甲板 ID 由「灯条颜色」+「CNN 的 3 个类别」决定（`Armor::get_id()`）：

| 灯条颜色 | CNN class_id | armor_id | 对应车 |
| --- | --- | --- | --- |
| 0 = 红 | 0 | 6 | R1 |
| 0 = 红 | 1 | 8 | R3 |
| 0 = 红 | 2 | 11 | R7（哨兵） |
| 1 = 蓝 | 0 | 0 | B1 |
| 1 = 蓝 | 1 | 2 | B3 |
| 1 = 蓝 | 2 | 5 | B7（哨兵） |

其余组合返回 `-1`，并且会被 `is_armor()` 丢弃。**目前只会输出上表这 6 个 ID**（不会出现 B2/B4/B5、R2/R4/R5）。换模型/换类别数时必须同步修改 `detector.cpp` 的 `get_id()` 与 `is_armor()` 里的类别上限。

## 使用方式

### 编译

```bash
source /opt/ros/humble/setup.bash          # 每个新终端一次
cd ~/DT46-V
colcon build --packages-select rm_interfaces rm_detector --symlink-install
source install/setup.bash
```

`rm_interfaces` 必须先于 `rm_detector` 编译（自定义消息依赖）。

### launch 启动

```bash
ros2 launch rm_detector detector.launch.xml
```

- 该 launch 只启动 `rm_detector_node` 一个节点，参数文件固定为 `config/detector_params.yaml`，**没有暴露任何 launch argument**。
- 要换参数文件（英雄/步兵/哨兵）请直接用整机 launch：`ros2 launch rm_vision_bringup hero.launch.xml` 等，见 `rm_vision_bringup` 的 README。

### ros2 run

```bash
# 用默认参数（会去找 share/rm_detector/model/mlp.onnx，通常需要显式指定模型）
ros2 run rm_detector rm_detector_node

# 指定参数文件（YAML 根键必须与节点名 rm_detector 一致）
ros2 run rm_detector rm_detector_node --ros-args \
  --params-file install/rm_detector/share/rm_detector/config/detector_params_sentry.yaml
```

### 动态调参

```bash
ros2 param list /rm_detector
ros2 param set /rm_detector detect_color 0     # 只找红灯条
ros2 param set /rm_detector display_mode true  # 打开调试图推流
ros2 param set /rm_detector cls_model_file RM_Armor_CNN_sim.onnx  # 热重载模型
```

### 最小调试组合

需要同时有图像和内参才有输出：

```bash
ros2 launch usb_camera usb_camera.launch.xml     # 或 hik_camera / mindvision_camera
ros2 launch rm_detector detector.launch.xml
ros2 topic echo /detector/armors_info
```

整机场景下由 `rm_vision_bringup` 的 `*_nav.launch.xml`（只跑视觉）或 `hik.launch.xml` / `mv.launch.xml`（含串口、IMU）负责组合。

## 依赖与前置条件

- ROS 包依赖：`rclcpp`、`sensor_msgs`、`std_msgs`、`cv_bridge`、`OpenCV`（需要 dnn/imgproc/calib3d 模块）、`ament_index_cpp`、`rm_interfaces`。
- 编译选项：`-Wall -Wextra -Wpedantic -O3 -march=native`，编译稍慢。
- 模型文件随包提交：`model/RM_Armor_CNN_sim.onnx`（约 453 KB），安装到 `share/rm_detector/model/`。
- `NumberClassifier` 用 `cv::dnn::readNetFromONNX` 加载（不是 ONNXRuntime）：先尝试 CUDA backend，失败自动退回 CPU。
- 上游数据源：必须有节点发布 `/image_raw`（BGR8）与 `/camera_info`。

## 注意事项 / 已知问题

1. **分类器不可用 = 检测不到任何装甲板**：`cls_model_file` 为空或模型加载失败时 `classifier_` 为 `nullptr`，`get_armor_result()` 不会填 `class_id`（默认 -1），`is_armor()` 的 `class_id ∈ {0,1,2}` 判断会丢弃全部候选，终端只会周期性打印 `No armors detected`。
2. **必须有 `/camera_info`**：内参首次解析后缓存；尚无内参时 PnP 返回无效深度，不输出有效位姿。
3. **`use_geometric_center: true` 会覆盖标定主点**：所有 yaml 当前都是 true，等于放弃标定得到的 `cx/cy`，只用图像几何中心。若要做高精度测距/换分辨率，注意这一点。
4. **调试图像很吃带宽**：`display_mode: true` 时 6 路图像都用 SensorDataQoS 发布，只是看调试图时记得关掉。
5. **丢帧策略**：`image_callback` 覆盖式保存最新帧、`processing_loop` 取走即置空，所以处理慢时直接丢帧而不是排队，不会"重复识别同一帧"。
6. **参数默认值与 YAML 不一致**（见参数表）：`ros2 run` 不带参数文件时行为和跑 launch 完全不同，排查问题时先确认参数从哪来。
7. 当前 PnP 对所有 id 使用同一套可配置灯条尺寸，不按编号推定大小板。ROS 的两个尺寸是启动参数，运行时修改会明确拒绝，需修改 YAML 后重启；原生 Apply 重建上下文后生效。
8. PnP 无可用解或缺少内参时返回 `(0, 0, -1, 0, 0, 0, 0)`，下游必须将 `dz = -1` 视为无效。
