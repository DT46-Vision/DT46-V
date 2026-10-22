# 无 ROS 自瞄核心

本目录直接编译原 `rm_detector` 的灯条检测、ONNX 分类、PnP，以及 `rm_tracker` 的 EKF、目标选择和弹道 LUT。无需 ROS、WSL、相机 SDK、串口或 `colcon`。原 ROS 节点保留为可选通信适配层；本次未在 ROS 环境重新编译。

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

## 仿真接入

仿真启动方式及验证结果由[工作区 README](../../README.md#dt46-v-与双相机)维护；`virtual_aim.py` 从 `StageCamera.read("onboard")` 采集 RGB，读取同一时刻自身光学姿态、标定和双光电门反馈，并转换角度误差驱动云台。

执行端是理想角度控制，尚无 MCU/PID、执行器延迟或限速模型。自动开火需显式启用，且同时满足 `TRACKING` 与原 `can_fire`，短时丢失期间仅保持预测瞄准。当前验证限定自身底盘静止、敌车静止或巡逻；移动自身、陀螺目标及实机效果未验证。装甲反馈来自仿真接触代理，不能解释为实车命中率。
