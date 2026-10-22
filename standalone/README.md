# DT46-V 原生视觉运行

仓库根 CMake 构建 C++20 运行程序。检测、ONNX 分类、PnP、EKF、目标选择与弹道 LUT 直接复用 `src/rm_detector`、`src/rm_tracker`；`project/modules/vision` 维护原生接口和调试显示，`project/platform` 维护本机通信与窗口控制，`tools` 维护离线工具。ROS 节点是可选适配层，不是原生程序的运行前提。

## 构建

需要 CMake 3.24+、Ninja、C++20 编译器，以及 OpenCV 4.12、Eigen 3.4。可使用现有原生安装，或通过 CMake 准备固定版本依赖：

```powershell
cmake -DWORK_DIR=<依赖目录> -DJOBS=4 -P cmake/bootstrap.cmake
cmake --preset release -DOpenCV_DIR=<依赖目录>/opencv -DEIGEN3_INCLUDE_DIR=<依赖目录>/deps/eigen-3.4.0 -DCMAKE_RUNTIME_OUTPUT_DIRECTORY=<仓库绝对路径>/build/native/bin
cmake --build --preset release --parallel 4
ctest --preset release
```

依赖下载及校验值由 [bootstrap.cmake](../cmake/bootstrap.cmake) 维护；本工程另固定 nlohmann/json 和 GLFW。OpenCV 使用 CPU DNN，并启用图像和视频读写。Windows 运行前，将 OpenCV 安装目录下的 `x64/mingw/bin`（或相应编译器目录）加入当前进程 PATH；程序与依赖应使用兼容的编译器。FFmpeg 插件随 OpenCV 安装，录制时需要该插件。Linux 视频后端依赖本机 OpenCV/FFmpeg 构建配置。

当前 Windows 原生测试已通过：标定 PnP/光学旋转、管线空图/时间戳/复位/参数原子更新、本机通信认证与长度限制、MCU/IMU 字节协议。媒体测试也已通过：20 视图已知参数标定恢复、20 帧 MP4 写入/读回及采样节奏；合成标定 RMS 为 0.000021 像素，不代表实机精度。40 帧仿真序列回放得到 31 帧检测、26 帧 TRACKING 和 8 帧录像；与迁移前接口比较的 1,914 个数值完全一致，比较不包含耗时、通信附加字段和调试日志快照。ROS 节点与实机串口未验证，本机没有 ROS 2 / WSL。

## 虚拟相机与独立调试

```powershell
build/native/bin/dt46_vision.exe --camera-session <相机会话.json> --output <结果目录>
build/native/bin/dt46_vision.exe --show-debug
build/native/bin/dt46_vision.exe --close-debug
build/native/bin/dt46_vision.exe --save-debug
```

仿真驾驶默认启动第一条命令对应的独立进程。识别默认在后台运行，调试窗口读取同一管线；关闭、重开显示不会重建检测器或追踪器。调试命令通过本工程 `.cache/virtual_camera/session.json` 找到当前任务，可用 `--debug-session` 指定实例。退出只清理自己创建的会话。相机正常关闭后视觉结束；协议错误与超时记录为失败。视觉断开不得影响驾驶，完整边界和操作验收见 [sim_vision 项目规范](../../README.md#项目规范工程边界与使用流程)。

`--profile infantry|sentry|hero` 选择原 YAML/ONNX；`--parameters` 加载调试窗口保存的 JSON；`--record result.mp4 --video-fps 20` 录制原跟踪/弹道叠加图；`--duration`、`--frames` 限制运行。采样不足时不会填入重复旧画面，所以 MP4 的固定帧率时长可能短于真实运行时长；采样时间以 `frames.jsonl` 为准。结果包含逐帧 JSON、结束报告及调试事件，录制和显示都不执行云台或开火指令。

调试窗口提供四格选择：原始 RGB、跟踪、弹道、result、crop_img、bin_img、img_armor、img_armor_processed、img_debug、gray；灰度图显示前转为 RGB。窗口包含相机、检测器、追踪器参数和 tracker 日志，可应用、保存、加载 JSON，保存当前图像与窗口截图。参数 Apply 明确重建上下文；失败保留原上下文。显示异常只关闭显示。

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

`aim` 提供当前帧原始解算：`valid`、`yaw_error_deg`、`pitch_error_deg`、`can_fire`。`valid` 表示本次弹道解算有效，与允许开火分开；每次 track 先清除，提前退出不会沿用旧值。顶层同名角度继续保留 EMA，供现有日志使用；执行端使用 `aim` 内的原始角，避免将不同相机姿态下的平滑误差混入当前帧。`AimResult` 新增有效性和原始角字段，C 接口使用者须用当前头文件重新编译。管线 `revision()` 在成功 reset/configure 后递增，失败不变；原生会话撤销旧 revision 的反馈，调试窗口开关不改变 revision。

`debug_image()` 返回独立内存的 RGB 或灰度图；须启用显示参数，无当前图像时返回空。跟踪和弹道图由 [debug_images.cpp](../project/modules/vision/debug_images.cpp) 按原快照绘制。红圈为含飞行时间/系统延迟的预测目标，绿十字为枪口相对目标，紫十字为原图像中心。绿十字不是弹道落点；HUD Pitch/Yaw 沿用原 tracker 指令，顶层结果角误差沿用 EMA 输出。投影使用完整 K/D。

## 本机相机接口 v1

[transport.hpp](../project/interfaces/transport.hpp) 维护原生 TCP 消息格式：回环地址、逐会话密钥、长度受限的 JSON 和原始 RGB 二进制数据。相机服务端属于 sim_vision，客户端属于 DT46-V。`next` 的可选结果字段返回视觉解算，由仿真侧根据驾驶输入执行；本接口不连接实车执行器。

| 请求 | 作用 |
|---|---|
| `describe` | 获取协议版本、相机参数及初始弹速/重力/阻力 |
| `next(after, status, result)` | 获取指定序号后的最新帧；暂无新帧时返回空；status 回报识别/显示状态，result 返回上一张已处理帧的紧凑自瞄结果 |
| `validate_camera(parameters)` | 只验证相机参数 |
| `configure_camera(parameters)` | 请求应用相机参数 |

帧元数据含 `time`、`K`、`D`、`optical`（4×4 相机到世界变换）、相机系 `muzzle_offset`、敌方 `color`、自身有效测速 `speed`、复位 `generation`、相机参数、帧编号和物理步编号。图像是 width×height×3 字节 RGB。分代变化清除历史，暂停不重复处理时间戳。

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
