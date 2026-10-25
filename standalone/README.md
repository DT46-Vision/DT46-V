# DT46-V 原生视觉运行

仓库根 CMake 构建 C++20 运行程序。检测、ONNX 分类、PnP、EKF、目标选择与弹道 LUT 直接复用 `src/rm_detector`、`src/rm_tracker`；`project/modules/vision` 维护原生接口和调试显示，`project/platform` 维护本机通信与窗口控制，`tools` 维护离线工具。ROS 节点是可选适配层，不是原生程序的运行前提。

当前工作分支为 `codex/aim-tracking-scenario`，从已整合成像及原生调试工作的 `codex/sim-vision-work@e0e0981` 分出，本次仅本地保存。原始起点是 GlennAde 的 `aim_test@3c41f0b02a05bf06ad28ca8440bc8099545498ab`（“修了一下平移限定”）。工作历史汇集原 `codex/cpp-migration` 的 14 个开发提交和 `codex/docs-aim-test-review` 的 6 个审查/修复提交；不以后来混入这些改动的上游合并提交作为原始基线。远端默认分支仍为 `stable`（2026-10-23 核对）。后续 Git 操作遵守[工作区分支隔离规范](../../README.md#git-分支隔离)。

此前工作分支汇集见[独立工作分支检查记录](../../outputs/branch_isolation_20261023/checks.json)；本次配套整合与原生回归结果见[整合收尾检查](../../outputs/integration_closeout_20261025/checks.json)。

## 原始 tracker 基线

按用户指定，`tracker.cpp` / `tracker.hpp`、`ekf.cpp` / `ekf.hpp`、ROS `node.cpp` 保持 GlennAde `3c41f0b` 的源码，忽略检出时的 CRLF/LF 差异。`original_tracker_baseline` 测试校验这五个文件；后续不能以修复、迁移或适配为由自动改变其逻辑。原算法行为由 [rm_tracker](../src/rm_tracker/README.md#数据流与算法)维护。

Windows 接入保留在原生适配层；`RmTF` 额外接收光学旋转和真实标定主点，ROS 调用仍走原默认路径。原 tracker 重力固定为 9.81 m/s²；配置其他重力时明确拒绝，不能声称已跟随仿真重力。弹速、阻力和枪口偏移继续读取相应反馈。每帧有效性由适配层读取原始弹道求解器的结果，不在 tracker 中新增状态或改变选板、EKF、弹道和发弹判据。

Git 历史改动、恢复范围、剩余检测器差异、功能审查及本次验证见 [2026-10-23 审查快照](../docs/reviews/native-parity-20261023/README.md)。旧 [aim_test 审查](../docs/reviews/aim-test-3c41f0b/README.md)记录的是另一条修复历史，不能作为当前 tracker 已采用这些修复的依据。

2026-10-23 交接访谈确认：平台大改期间以本节原 tracker 为默认基线。PR3 和 `codex/sim-vision-tracker-changes@032dfb1` 保留为后续优化计划，待平台大改完成后再开始算法优化；本次不切换默认 tracker。最终将经过优化、验证的快照算法改动整合回 `codex/sim-vision-work`，形成统一工作版本。合并结果保留当前适配层的目标颜色过滤和下文的输入翻转隔离；原快照继续作为历史对照保留。快照的算法与适配层差异记录在该分支的 `standalone/README.md`，优化验证需先统一输入与验收条件。本次仅确认后续整合方向，未执行算法合并。

## 构建

需要 CMake 3.24+、Ninja、C++20 编译器，以及 OpenCV 4.12、Eigen 3.4。可使用现有原生安装，或通过 CMake 准备固定版本依赖：

```powershell
cmake -DWORK_DIR=<依赖目录> -DJOBS=4 -P cmake/bootstrap.cmake
cmake --preset release -DOpenCV_DIR=<依赖目录>/opencv -DEIGEN3_INCLUDE_DIR=<依赖目录>/deps/eigen-3.4.0 -DCMAKE_RUNTIME_OUTPUT_DIRECTORY=<仓库绝对路径>/build/native/bin
cmake --build --preset release --parallel 4
ctest --preset release
```

依赖下载及校验值由 [bootstrap.cmake](../cmake/bootstrap.cmake) 维护；本工程另固定 nlohmann/json 和 GLFW。OpenCV 使用 CPU DNN，并启用图像和视频读写。Windows 运行前，将 OpenCV 安装目录下的 `x64/mingw/bin`（或相应编译器目录）加入当前进程 PATH；程序与依赖应使用兼容的编译器。FFmpeg 插件随 OpenCV 安装，录制时需要该插件。Linux 视频后端依赖本机 OpenCV/FFmpeg 构建配置。

当前构建提供 6 项原生测试：原始 tracker 基线、标定 PnP/光学旋转、管线、通信、MCU/IMU 字节协议及媒体。最新执行结果与限制见[本次审查](../docs/reviews/native-parity-20261023/README.md#验证结果)。此前迁移时的 40 帧回放和 1,914 个数值对照属于恢复 tracker 之前的历史结果，不能证明当前分支全链路与原 ROS 等价。ROS 节点与实机串口未验证，本机没有可用的 ROS 2 / WSL 环境。

## 虚拟相机与独立调试

```powershell
build/native/bin/dt46_vision.exe --camera-session <相机会话.json> --output <结果目录>
build/native/bin/dt46_vision.exe --show-debug
build/native/bin/dt46_vision.exe --close-debug
build/native/bin/dt46_vision.exe --save-debug
```

仿真驾驶默认启动第一条命令对应的独立进程。识别默认在后台运行，调试窗口读取同一管线；关闭、重开显示不会重建检测器或追踪器。调试命令通过本工程 `.cache/virtual_camera/session.json` 找到当前任务，可用 `--debug-session` 指定实例。退出只清理自己创建的会话。相机正常关闭后视觉结束；协议错误与超时记录为失败。视觉断开不得影响驾驶，完整边界和操作验收见 [sim_vision 项目规范](../../README.md#项目规范工程边界与使用流程)。

`--profile infantry|sentry|hero` 选择原 YAML/ONNX；`--parameters` 加载调试窗口保存的 JSON；`--record result.mp4 --video-fps 20` 录制原跟踪/弹道叠加图；`--duration`、`--frames` 限制运行。采样不足时不会填入重复旧画面，所以 MP4 的固定帧率时长可能短于真实运行时长；采样时间以 `frames.jsonl` 为准。结果包含逐帧 JSON、结束报告及调试事件，录制和显示都不执行云台或开火指令。

未指定 `--parameters` 或文件内容为 `{}` 时，原生视觉启动会加载 [virtual_camera.json](virtual_camera.json)，通过现有相机接口应用曝光（微秒）和相对增益，并覆盖原生 tracker 叠加图的 `text_size`；两种仿真相机预设共用该启动覆盖，标定和其他成像参数仍由各自预设提供。非空 `--parameters` 文件完整替代这一默认覆盖，保持已有调参文件的行为。相机组仅用于连接后的视觉相机，不写回仿真预设或 ROS 驱动 YAML，也不作用于操作手图传；tracker 字号覆盖不改写 ROS 共用 YAML。没有启用视觉相机/DT46 的会话不会因此开启识别。离线回放仍使用已经采集的图像，调曝光须先重新采图。

原生启动配置降低默认相对增益，以匹配仿真曝光前的独立灯条发光强度，避免继续把背景一起放大；数值以 `virtual_camera.json` 为准，不等同于实拍海康界面的增益 16.0。当前红蓝检出、跟踪和相机预设验证由[仿真灯条成像](../../README.md#画质灯条与反光)统一维护；本次未改检测器、分类器或原 tracker。此前只提高整幅图像增益的[历史调参记录](../../outputs/led_emission_20261024/previous_gain_tuning.json)仅证明当时蓝靶成功、红靶未检出，不能代表本轮模型。

调试窗口以 [Kielas_Vision.perspective](../Kielas_Vision.perspective) 实际启用的四个 `rqt_image_view` 和一个 `rqt_reconfigure` 为功能基准，保留原生四格 UI。默认四图为 `/detector/result`、`/tracker/ballistic_img`、`/detector/img_armor_processed`、`/detector/bin_img`；仍可切换原始 RGB、tracking_state_img、crop_img、img_armor、img_debug、gray。灰度图显示前转为 RGB。各格提供平滑、1:1 滚动查看、网格和 90° 步进旋转；这些只影响查看。默认关闭平滑、1:1、网格和旋转，与保存的 rqt 配置一致。

右侧仍分相机、检测器、追踪器三页，包含 tracker 日志、参数应用/保存/加载 JSON、保存当前图像与窗口截图。顶栏 `UI font` 独立调整面板字号，行高、侧栏宽度和图像工具栏随之缩放；只在当前窗口有效，不保存到算法参数，也不重置 tracker。布尔参数使用复选框；检测器 `cls_model_file` 可编辑字符串，JSON 沿用顶层 `model` 字段，模型相对 `src/rm_detector/model` 解析。Apply 重建上下文，失败保留原上下文；不会写回源 YAML。图像查看选项仅在当前窗口有效，不随参数 JSON 保存。显示异常只关闭显示。

| 原 rqt / 节点功能 | 原生实现与适配边界 |
|---|---|
| MindVision 曝光、增益、RGB、AE/AWB、饱和度、gamma | 使用相机服务的同名控制；成像近似与开销边界见 [仿真相机说明](../../README.md#仿真时序与采图) |
| 图像旋转 | 工作分支仅保留各调试画面的显示旋转；算法始终接收原图及对应 K/D、光学姿态和枪口偏移。识别前旋转整图的 `flip_image` 已移至 `codex/camera-input-flip-experiment`（保留点 `bd98ec6`），该分支保留了翻图后坐标未同步的已知问题，仅供后续研究。相机面板和新结果元数据不再提供该选项；CLI 参数文件及调试窗口 Load 均拒绝包含 `camera.flip_image` 的旧配置，包括 false/0，需删除该键后重新加载。ROS 实机驱动的同名功能未改动。 |
| 检测器灯条/装甲/ROI/阈值/模型/显示参数 | 对接原检测器；`detect_color` 支持 0 红、1 蓝、2 两色，另以 -1 保持原生默认的目标颜色跟随；`print_period_ms` 控制原生会话处理 FPS 的终端日志周期，不代表独立 detector/tracker 线程速率 |
| tracker EKF、匹配、半径、延迟、弹道、旋转阈值 | 使用原始核心参数；`target_color` 支持红/蓝，默认 `follow_decision=true` 从仿真反馈取色，关闭后使用手动值；送入 tracker 前按原 ROS 节点过滤颜色，即使检测页选择两色也不追踪非目标颜色 |
| `display` / `display_fps_limit` / `text_size` | 控制跟踪和弹道图生成、约 30 Hz 的调试图更新限制、文字缩放；关闭 `display` 停止这两路图和其录像，识别继续。解除刷新限制会增加图像生成开销，窗口仍按约 30 Hz 绘制 |
| 相机名称/标定地址、枪口位置、IMU 安装修正 | 原生相机标识、K/D、光学姿态和枪口偏移由采样反馈提供；面板说明来源并显示当前枪口偏移。`rotation_rpy_*` 是 ROS 的 IMU 适配参数，不再次叠加；`use_geometric_center` 固定为 false，保留标定主点 |
| ROS QoS、图像点击发布、节点选择 | 由原生相机会话和三页参数分类适配；不伪造 ROS 参数服务器或点击话题。原保存布局未启用点击发布；其他未启用的 rqt 插件不构成本页基准 |

参数枚举依据 [相机节点](../src/rm_vision_ros2_mindvision_camera/src/mv_camera_node.cpp)、[检测节点](../src/rm_detector/src/node.cpp)、[跟踪节点](../src/rm_tracker/src/node.cpp) 的实际声明与回调，不把 YAML 中未声明的键算作 rqt 功能。原生测试和 Windows 操作流程的历史记录见[审查快照](../docs/reviews/native-parity-20261023/README.md#验证结果)。此前的[参数文件界面检查](../../outputs/rqt_parity_20261023/save_verified/checks.json)通过 Load → Save → Load，保留模型字符串和布尔类型；[显示检查](../../outputs/rqt_parity_20261023/acceptance/checks.json)包含当时的输入翻转及跟踪图关闭/恢复。这些是恢复 tracker、隔离输入翻转之前的历史结果，不代表当前功能或算法验收。本机证据不纳入 Git；当前检查使用本页构建/CTest 命令及 sim_vision 的 Windows 操作流程，界面检查使用输出目录的 `vision_parameters.json`，验证显示旋转、`tracker.display`、保存/加载以及旧翻转配置的拒绝。ROS rqt 本体未在本机运行验证。

2026-10-23 输入翻转隔离验证：6 项原生 CTest 全部通过；`native_pipeline` 检查所有 RGB 字节及 K/D、光学姿态、枪口偏移原样交付，拒绝旧翻转参数且保持当前配置，`original_tracker_baseline` 确认五个核心文件仍为原版。沿用审查快照中的 Windows 流程命令，本次 [workflow_44312](../../outputs/cpp_migration/workflow_44312/checks.json) 检查 1,016 帧，许可发弹 2 发、视觉断开后 0 发；调试开关及快照成功，保存参数不含 `flip_image`，截图保留每格显示旋转入口。未单独点击所有旋转角度及 GUI Load 文件对话框；旧参数拒绝由原生测试覆盖。本次未验证 ROS 或实车。

## 管线接口与坐标

[AimPipeline](../project/modules/vision/pipeline.hpp) 按序处理一路相机；同一实例不支持并发调用。`reset()` 清除追踪、PnP 历史及时间戳，析构释放算法上下文。`configure()` 原子重建，成功后清除历史，不改写源 YAML。允许的参数由实现中的白名单维护。

| 输入 | 约定 |
|---|---|
| `rgb` | 实际采样、原始畸变 RGB8；内部转换为检测器使用的 BGR |
| `timestamp` | 单调递增秒；暂停不重复提交，复位先清除历史 |
| `K` / `D` | 当前输出尺寸的 3×3 内参、五项 plumb_bob 畸变；保留真实主点 |
| `optical_rotation` | 光学 right/down/forward 轴到世界轴的 3×3 旋转 |
| `muzzle_offset` | 相机光心指向枪口出口，使用相机坐标、米 |
| `target_color` | 蓝色 1、红色 0 |
| `measured_speed` | 有效测速，米/秒；未获得传 0，沿用 tracker 配置弹速，原核心仅采用大于 10 m/s 的反馈 |

输出包含装甲编号、分类分数、像素四角、相机系米制位置、追踪状态、预测状态、角度误差（度）及 `can_fire`。`position_camera_origin_world_axes_m` 以相机为原点，按世界轴表达；没有补偿自身底盘平移与光心移动，不能当作世界绝对位置。仿真反馈覆盖对应硬件输入，已给出的光学旋转不再叠加 IMU 安装修正。输入不含对手真值。

非 `LOST` 帧的只读 `tracker_debug.ekf_state` 顺序为 `[xc,vx,yc,vy,za,vz,yaw,vyaw,r]`，位置单位米、速度米/秒、角度弧度、角速度弧度/秒。车心 XY 与装甲基准高度 za 均以采样时的相机光心为原点、按世界轴表达；转换世界位置只加 `input.optical_transform` 的平移，不能再次旋转。速度包含相机原点运动的影响。适配层同时导出原 `RenderSnapshot` 的 `ekf_second_radius_m`（另一对板半径）与 `ekf_alternate_height_m`（相邻板高度差 dz），不修改 tracker。四板模型为 `yaw_i=yaw-iπ/2`、`p_i=(xc-r_i cos(yaw_i), yc-r_i sin(yaw_i), z_i)`，偶数板取 r、za，奇数板取另一半径、za+dz。此状态重建与最终弹道目标不同；仿真同帧真值和评测流程见[自瞄验证场景](../../README.md#自瞄运动与预测验证场景)。

`aim` 提供当前帧原始解算：`valid`、`yaw_error_deg`、`pitch_error_deg`、`can_fire`。`valid` 表示本次弹道解算有效，与允许开火分开。适配层在调用 `track` 前清空仅供渲染的目标缓存；若本帧产生枪口目标，再调用原本无状态的 `solve_ballistic` 读取有效标记并检查有限角度。提前退出时不会导出旧帧有效性；额外调用只有原查表和三角运算，不修改核心状态。顶层同名角度继续保留 EMA，供现有日志使用；执行端使用 `aim` 内的原始角，避免将不同相机姿态下的平滑误差混入当前帧。`AimResult` 新增有效性和原始角字段，C 接口使用者须用当前头文件重新编译。管线 `revision()` 在成功 reset/configure 后递增，失败不变；原生会话撤销旧 revision 的反馈，调试窗口开关不改变 revision。

图像处理对齐原 ROS 链路：相机发布 `image_raw`，检测器和 tracker 调试订阅原图；所核对的 `mv.launch.xml` / `infantry.launch.xml` 没有图像矫正节点。检测前不调用整图 `undistort`，PnP 则把原始像素和 K/D 传给 `solvePnPGeneric`，在几何求解中补偿畸变。原生调试页继续显示原图及其检测/跟踪/弹道叠加，新 UI 保留；没有另建仿真专用的拉直图或识别算法。源码依据为 [ROS 检测节点](../src/rm_detector/src/node.cpp)、[原 PnP](../src/rm_detector/src/pnp.cpp) 和 [原生显示入口](../project/modules/vision/debug_window.cpp)。合成中心/边缘对照由 `calibrated_pnp_and_optical_rotation` 检查；这不等于实机镜头标定验收。

`debug_image()` 返回独立内存的 RGB 或灰度图；须启用显示参数，无当前图像时返回空。跟踪和弹道图由 [debug_images.cpp](../project/modules/vision/debug_images.cpp) 按原快照绘制，叠加文字使用抗锯齿、随字号加粗的笔画及黑色描边；开火许可为绿色，等待开火为黄色。红圈为含飞行时间/系统延迟的预测目标，绿十字为枪口相对目标，紫十字为原图像中心。绿十字不是弹道落点；HUD Pitch/Yaw 沿用原 tracker 指令，顶层结果角误差沿用 EMA 输出。投影使用完整 K/D。

## 本机相机接口 v1

[transport.hpp](../project/interfaces/transport.hpp) 维护原生 TCP 消息格式：回环地址、逐会话密钥、长度受限的 JSON 和原始 RGB 二进制数据。相机服务端属于 sim_vision，客户端属于 DT46-V。`next` 的可选结果字段返回视觉解算，由仿真侧根据驾驶输入执行；本接口不连接实车执行器。

| 请求 | 作用 |
|---|---|
| `describe` | 获取协议版本、相机参数及初始弹速/重力/阻力 |
| `next(after, status, result)` | 获取指定序号后的最新帧；暂无新帧时返回空；status 回报识别/显示状态，result 返回上一张已处理帧的紧凑自瞄结果 |
| `validate_camera(parameters)` | 只验证相机参数 |
| `configure_camera(parameters)` | 请求应用相机参数 |

帧元数据含 `time`、`K`、`D`、`optical`（4×4 相机到世界变换）、相机系 `muzzle_offset`、敌方 `color`、自身有效测速 `speed`、复位 `generation`、相机参数、帧编号和物理步编号。可选 `camera_settings_version`、`sampling_target_hz`、`sampling_effective_limit_hz` 由适配层原样写入日志；相机配置成功后只接收对应新版本图像。图像是 width×height×3 字节 RGB。分代变化清除历史，暂停不重复处理时间戳。

`result` 为 `null` 或 `{schema:1, frame_number, state_tick, reset_generation, simulation_time_s, camera, robot_id, state, tracked_id, aim}`；具体构造入口为 [aim_feedback](../project/modules/vision/virtual_camera.cpp)。启动、上下文重建或无结果时发 `null`。旧客户端缺省该字段仍可读图，但不会提供自瞄许可。服务端要求结果与本连接最后交付帧的身份、代号、采样时间完全一致，使用服务端保留的原帧姿态，不接受客户端另传的姿态；重复帧不刷新许可，错误反馈、空结果、相机配置变更和断开清除缓存。

`aim` 两个误差分别为相机 `atan2(x,z)` 和 `atan2(y,z)` 的度数，正方向为图像向右/向下，已含弹道补偿。执行端以该帧 `optical` 的旋转 `R` 计算 `d = R × [tan(yaw), tan(pitch), 1]`，再得到世界 yaw=`atan2(d.y,d.x)`、pitch=`atan2(d.z,hypot(d.x,d.y))`；三角函数输入先转弧度。它们不是两个 Euler 旋转，也不能每个物理步累加到最新云台角上。交付时间与采样时间共同限制有效期，具体门控和配置由 [sim_vision 自瞄操作](../../README.md#右键自瞄与发弹)维护。

可选 `appearance` 记录该帧的仿真画质、灯条和表面显示设置，原样保留在识别日志及调试截图元数据中，不参与识别决策。画质控制与视觉采样验收由 [sim_vision 画质说明](../../README.md#画质灯条与反光)维护。

## 离线回放、录像与标定

仿真使用 `sim_sequence` 导出 PNG 与 `frames.jsonl`，DT46 独立读取：

```powershell
build/native/bin/dt46_replay.exe --sequence <序列目录>/frames.jsonl --output <结果目录> --record <结果目录>/tracking.mp4 --video-fps 20 --dump-debug
```

回放使用真实图像及其同次采样的相机/自身反馈；支持 `--profile`、`--parameters`、`--model`，输出逐帧结果、识别截图、末帧和报告。图像路径相对序列文件，时间戳与标定不能与其他帧混用。相机成像参数必须在生成序列前设置；回放不能追溯改变原图曝光。

相机标定使用不同位置、距离和倾角的棋盘图像，至少五张同分辨率有效图；`columns/rows` 是内角点数，`square` 单位米：

```powershell
build/native/bin/dt46_calibrate.exe --images <棋盘图片目录> --columns 8 --rows 6 --square 0.025 --name camera --output camera_info.yaml
```

输出标准 ROS camera_info YAML 和逐视图重投影误差报告。误差小不等于实机标定准确；应检查姿态覆盖、棋盘尺寸和独立图像投影。不会自动改写驱动配置。

ROS 环境提供同一标定代码，以及图像/MP4 采集入口：

```bash
colcon build --packages-select dt46_tools
source install/setup.bash
ros2 run dt46_tools record_images --topic /image_raw --images /tmp/board --interval 1 --count 25
ros2 run dt46_tools calibrate_camera --images /tmp/board --output /tmp/camera_info.yaml --columns 8 --rows 6 --square 0.025
ros2 run dt46_tools record_images --topic /tracker/ballistic_img --out /tmp/tracker.mp4 --fps 20 --idle 8
```

图像采集按墙钟间隔保存新消息；录像逐消息写入，`fps` 指定回放帧率，`idle` 在已接收图像后超时退出。Ctrl+C 关闭视频。检查标定结果后，将 YAML 放到对应相机包配置位置，重新安装包并重启驱动。ROS 入口尚未在本机编译或运行。

哨兵噪声参数变体工具保留 YAML 其余内容与注释：

```powershell
build/native/bin/dt46_sentry_variant.exe input.yaml output.yaml 20 0.01 100 80
```

四个数依次为 `ekf_QR_q_xyz`、`ekf_QR_r_xyz_factor`、`ekf_QR_q_yaw`、`ekf_QR_q_r`；ROS 安装入口为 `ros2 run dt46_tools make_sentry_variant`。硬件协议与参数分别由 [rm_serial](../src/rm_serial/README.md)、[dm_imu](../src/rm_vision_ros2_dm_imu/README.md) 维护，实车录制场景由 [bag 检查表](../bags/RECORD_CHECKLIST.md) 维护。
