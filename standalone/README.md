# DT46-V 非 ROS 工程

## C++ 运行层迁移（进行中）

独立开发分支 `codex/cpp-migration` 已增加仓库根 CMake 入口，以及 `project/modules/vision/pipeline.hpp` 的 C++ 管线。它直接链接现有算法库，读取原 profile YAML/ONNX，提供参数原子更新、单调时间戳检查、复位、检测跟踪结果和调试图像。新代码使用 C++20；Windows 显式启用标准库数学常量兼容定义，使 MSVC 与 MinGW 的严格标准模式均能编译原算法。

```powershell
cmake --preset release -DOpenCV_DIR=<原生OpenCV目录> -DEIGEN3_INCLUDE_DIR=<Eigen包含目录>
cmake --build --preset release --parallel 4
ctest --preset release
```

运行测试时应将 OpenCV 动态库目录加入当前进程 PATH。当前三项 CTest 已通过：原有标定 PnP/光学旋转；C++ 管线的空图处理、重复时间戳拒绝、复位和无效配置更新保留原上下文；1280×1024 RGB 通信、认证和消息长度限制。尚未验证完整图像序列与旧管线的逐帧一致性；实机串口/IMU 和其余工具仍待迁移。

原生运行入口为 `build/native/bin/dt46_vision.exe --camera-session <仿真会话JSON> --output <结果目录>`。默认后台识别；同目录另运行 `build/native/bin/dt46_vision.exe --show-debug` 打开当前任务的调试窗口，关闭窗口继续识别。`--close-debug` 关闭显示，`--save-debug` 保存当前图像与参数；`--debug-session` 可明确指定控制会话文件。默认会话位置为工程 `.cache/virtual_camera/session.json`，退出时仅清理自己创建的会话。仿真关闭后视觉通过连接关闭正常退出；超时或协议错误独立记录，不能当成正常退出。

窗口使用固定 GLFW 3.4 所附 Nuklear C/C++ 界面，提供原始图、弹道图、跟踪图、检测中间图及数字裁剪等十种内容，四格选择、相机/检测/跟踪参数、JSON 保存加载和原 tracker 日志。灰度图在显示时转换为 RGB。参数更改使用原管线的原子重建，模拟物理输入只读；关闭/重开窗口不重建管线，窗口异常也不结束识别。结果为逐帧 `frames.jsonl`、`report.json` 和调试操作事件；保存包含图像、结果、参数及窗口截图。完整仿真联调结果由[工作区迁移说明](../../README.md#c-迁移进行中)维护。下文旧运行入口仍处于替换清理阶段。

本目录直接编译原 `rm_detector` 的灯条检测、ONNX 分类、PnP，以及 `rm_tracker` 的 EKF、目标选择和弹道 LUT。无需 ROS、WSL、相机 SDK、串口或 `colcon`。原 ROS 节点保留为可选通信适配层；本次未在 ROS 环境重新编译。

DT46-V 是独立的视觉/自瞄工程，`standalone` 是它的非 ROS 运行方式，职责包括视觉算法、参数、日志和原有调试界面。sim_vision 是提供虚拟相机的仿真工程，不是自瞄的运行主体。跨工程职责、窗口关系及验收要求统一见[项目规范](../../README.md#项目规范工程边界与使用流程)。

## 构建

需要 Python 3.12+、NumPy、PyYAML，以及 CMake、Ninja、C++17 编译器。本机使用已有的 `D:/msys2/ucrt64/bin`，脚本仅在当前进程补充 PATH。工作区根目录执行：

```powershell
python DT46_V/standalone/build.py --work-dir outputs/native_aim --jobs 4
```

脚本从官方源下载固定版本 OpenCV/Eigen，并核对 [build.py](build.py) 中的 SHA256；仅编译所需 OpenCV 模块及其依赖，使用 CPU DNN。所有下载、构建和动态库保存在指定工作目录，不安装系统软件。构建结束自动运行 CTest，检查非居中内参下的 PnP 和光学旋转往返。

默认独立使用时，构建脚本和 Python 接口均使用视觉仓库内 `.cache/native`；仿真入口显式使用上面的 `outputs/native_aim`。Windows 动态库为 `build/dt46_aim.dll`。

## 输入输出

[aim.py](aim.py) 的 `AimPipeline(work_dir, profile="infantry", debug=False)` 通过 ctypes 调用 [aim.h](aim.h) 的 C ABI。`profile` 可选 infantry、sentry、hero，读取对应原始 YAML 和 ONNX；这表示参数入口可用，不代表三个车型均完成效果验收。一个实例按时间顺序处理一路相机，不支持同实例并发；用上下文管理器关闭，`reset()` 清除跟踪、PnP 历史和时间戳。

`process(rgb, timestamp, K, D, optical_rotation, muzzle_offset, target_color, measured_speed=0)`：

| 输入 | 约定 |
|---|---|
| `rgb` | 实际相机原始畸变图，`H×W×3 uint8 RGB`；内部转成原检测器的 BGR |
| `timestamp` | 单调递增仿真秒，暂停不重复提交；复位先调用 `reset()` |
| `K` / `D` | 当前输出尺寸的 `3×3` 内参与五项 plumb_bob 畸变；保留真实主点，不替换成图像中心 |
| `optical_rotation` | 相机 right/down/forward 轴到世界轴的 `3×3` 旋转 |
| `muzzle_offset` | 从相机光心指向枪口出口的向量，使用相机坐标，单位米 |
| `target_color` | 敌方颜色：蓝色 `1`，红色 `0` |
| `measured_speed` | 有效测速，米/秒；未获得时传 `0`，使用原 tracker 配置弹速。沿用原节点仅接受大于 10 m/s 的反馈 |

返回原识别器的装甲编号、分类分数、四角像素、相机系位置（米），以及跟踪状态、预测位置/速度、角度误差（度）和 `can_fire`。`position_camera_origin_world_axes_m` 以相机为原点、按世界轴表达，沿用原核心的旋转坐标模型；**没有补偿自身底盘平移与光心移动，不能将其当世界绝对位置**。

检测和跟踪的数值参数从所选 profile 读取，模型从原 detector 配置引用。仿真提供的 K/D、旋转、枪口几何、目标色和有效弹速覆盖对应硬件输入；IMU 安装修正不重复作用于已经给出的光学旋转。状态机、EMA 和时间步长保护按原节点实现；测得弹速变化超过原节点使用的阈值才重建 LUT。没有目标真值注入或新增在线优化器。仿真弹道参数及近距离补偿由[工作区发弹说明](../../README.md#发弹与测速)维护。

## 调试、参数与日志

`configure(detector={...}, tracker={...}, model=...)` 原子重建上下文，成功后清除跟踪/PnP 历史，失败保留原上下文；允许的数值参数见 `aim.py` 的白名单。不会改写源 YAML。相机硬件输入、真实主点及光学旋转由调用端提供，不用原 `use_geometric_center` 覆盖实际 K。`debug_image(name)` 返回拥有独立内存的 RGB 或灰度图；须开启 `debug`/`display_mode`，关闭显示或当前无装甲裁剪时返回 `None`，关闭 ROI 时 `crop_img` 沿用原核心的 1×1 占位。

| 原 ROS/rqt 内容 | 原生入口 |
|---|---|
| detector 的 result、crop_img、bin_img、img_armor、img_armor_processed、img_debug | 同名 `debug_image()`；额外提供 gray |
| tracker 的 tracking_state_img、ballistic_img | [debug.py](debug.py) 中的 `tracking_image()`、`ballistic_image()` |
| tracker 预测装甲、枪口目标、旋转状态与容差 | 返回值 `tracker_debug`，直接读取 `get_render_snapshot()` |
| tracker 状态切换/诊断日志 | `tracker_debug.logs`；逐帧 JSON 与本地调试面板显示 |
| rqt 参数修改 | `configure()`；本地窗口提供编辑、应用、JSON 保存/加载 |
| bag 图像录像脚本 | [工作区原生启动入口](../../README.md#dt46-v-与双相机)的 MP4 与逐帧 JSON |

红圈是原快照中包含飞行时间/系统延迟预测的目标，绿十字是原 rqt 枪口相对目标，紫十字保持原图像中心含义。绿十字不是“弹道落点”；HUD 的 Pitch/Yaw 显示原 tracker 指令，返回值的角误差是用于执行的 EMA 输出。投影使用当前原始畸变图的完整 K/D，避免在画面边缘错位。`tracking_image` 显示原快照装甲 yaw（弧度），`observer_image` 将同一预测点转换到另一自身相机，均不读取敌方真值。

原相机 SDK、串口、IMU 驱动和 ROS topic/rqt 进程不进入原生依赖。它们在仿真中的输入/输出替代关系、模拟曝光及实际验证范围见[工作区说明](../../README.md#dt46-v-与双相机)；原 ROS launch、bag 回放和实机通信仍需各自环境，不作为本地运行前提。

## 虚拟相机与独立调试窗口

[virtual_camera.py](virtual_camera.py) 是本工程的非 ROS 运行入口。从 **DT46_V 工程目录**运行：

```powershell
python -m standalone.virtual_camera --port <虚拟相机端口> --key <本机会话密钥> --work-dir <原生构建目录>
```

仿真驾驶入口默认启动这个独立进程，并提供该会话的相机端口和密钥；手动接入时使用同一组参数。`--work-dir` 缺省为本工程 `.cache/native`；当前仿真会话显式使用已有 `outputs/native_aim` 构建。可使用 `--profile infantry|sentry|hero` 选择原配置。本模块不导入仿真项目，不创建或控制车辆。

此 Python 入口使用 NumPy、PyYAML、OpenCV Python、Pillow 和 Tk；不加载 ROS 或相机硬件 SDK。本机所需依赖已具备。

识别默认后台运行，**调试窗口默认关闭**。想看正在运行的自瞄时，在 DT46_V 工程目录执行：

```powershell
python -m standalone.debug_window
```

[debug_window.py](debug_window.py) 通过本工程 `.cache/virtual_camera/session.json` 向已有自瞄任务请求显示；不会新建检测器或跟踪器。可使用 `--session` 指定其他运行实例的会话文件。关闭调试只销毁窗口；重新打开读取同一个 `AimPipeline` 的当前输出，保留跟踪状态。参数 Apply 才会按原接口明确重建上下文。

[vision_debug.py](vision_debug.py) 维护 DT46-V 原有四格图像、预瞄、日志和参数显示；不含驾驶、暂停或场景操作。相机曝光等参数先通过虚拟相机接口验证，再请求仿真应用。原始 RGB、K/D、PnP 和 tracker 继续沿用本工程的正常视觉链路；不会为了显示另跑识别或从仿真读取目标真值。

### 本机虚拟相机接口 v1

当前使用 Python `multiprocessing.connection` 的本机回环连接与逐会话密钥，仅连接双方本机 Python 进程。仿真是相机数据提供者，DT46-V 是客户端。相机接口不携带车辆执行指令。

| 请求 | 数据与作用 |
|---|---|
| `describe` | 读取版本 `protocol=1`、模拟相机参数及初始弹速/重力/阻力输入 |
| `next(after, status)` | 获取序号大于 `after` 的最新帧，暂无新帧返回空；`status` 仅回报本进程识别/显示状态 |
| `validate_camera(parameters)` | 验证模拟相机参数，不改变运行设置 |
| `configure_camera(parameters)` | 请求应用有效的相机设置，返回生效参数 |

每帧含 `image`（拥有独立内存的 uint8 RGB）、采样仿真秒 `time`、`K`（3×3）、`D`（五项）、`optical`（相机到世界的4×4变换）、相机系 `muzzle_offset`、敌方 `color`、有效测速 `speed`、`generation`、采样相机设置及帧/物理步编号。输入只使用图像与自身反馈；`generation` 变化时清除跟踪历史，暂停时不重复提交相同时间戳。处理核心的单位与坐标约定见上文输入输出表。

逐帧结果在 `--output` 指定目录流式写入 `frames.jsonl`，另存首次原图及结束汇总。默认输出为本工程 `.cache/virtual_camera`；仿真会话可明确指定其输出位置。工程之间的职责和验收要求见[项目规范](../../README.md#项目规范工程边界与使用流程)，本次联合运行记录见[仿真接入说明](../../README.md#dt46-v-与双相机)。
