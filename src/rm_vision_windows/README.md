# Windows Python 外层 / C++ 核心

本包属于视觉仓库的并行分支 `codex/python-outer`，从个人分支 `codex/sim-vision-work@a75f416` 分出。保留 aim_test 的 ROS 工作空间结构，把 Windows 仿真适配集中在一个新增包中。仿真程序、配置和协议不修改，也不在仿真仓库维护第二份视觉实现。

## 项目结构与来源

```text
DT46_V/
  src/
    rm_DT46/launch/*.launch.py          原 ROS bringup
    rm_detector/                       C++ 检测 / 分类 / PnP + ROS 节点
    rm_tracker/                        C++ 跟踪 / 弹道 + ROS 节点
    rm_interfaces/msg/                 原 ROS 消息
    rm_serial/rm_serial/                原 Python 串口节点
    rm_vision_ros2_dm_imu/dm_imu/        原 Python IMU 节点
    rm_vision_ros2_*_camera/             原相机驱动与 Python launch
    rm_vision_windows/                  本 Windows 适配包（COLCON_IGNORE）
      pyproject.toml                   Python 包依赖及命令入口
      setup.ps1                        环境准备入口
      rm_vision_windows/
        aim.py                         ctypes 调用现有 C ABI
        camera.py / transport.py        Windows 图像、姿态、决策输入与结果回传
        virtual_camera.py               启动、生命周期、参数版本和录制
        debug_window.py / vision_debug.py / debug.py
      tests/                           边界、来源及真实仿真验收
  standalone/                          C++ 核心的独立 CMake / C ABI
  .venv/                               本项目的 Python 运行环境
  .deps/                               本项目的原生依赖
  .local/                              本机路径、调试会话
  build/                               可重建的编译输出
  outputs/                             日志、截图、实验结果
```

ROS 文件从本地 `origin/aim_test@96bc416` 按原路径导入；具体范围和校验入口见 [source_audit.py](tests/source_audit.py)。这是选择性导入 ROS 接入文件，不是合并该分支上的后续算法优化。串口、IMU、消息、launch 的源码及其元数据保持来源版本；相机 SDK 不在本次替换。检测器 ROS 节点恢复原文件，`pnp.hpp` 只增加原 CameraInfo / logger 调用形式的兼容入口。受保护的五个 tracker 文件保持 [原始 tracker 基线](../../standalone/README.md#原始-tracker-基线)。

Python 数组包装及调试绘图从此前 ROS-free Python 版本 `a99e78b^:standalone/` 迁回本包，再适配当前 C ABI 和 Windows 通信。本包不是 ROS/DDS 实现；Windows 不安装 ROS，也不把 ROS 串口节点改为 Windows 节点。

根 README 的“全部由 C++ 实现”简介属于此前原生分支，不能描述此并行分支；该含图文档按协作约定未编辑。本页维护 Python 外层的结构与流程，[standalone](../../standalone/README.md) 维护共用 C++ 核心、消息和坐标契约。

## 环境放在哪里

| 内容 | 位置与维护方式 |
|---|---|
| 长期开发工具 | 电脑安装的 Git、CMake 3.24+、C++20 编译器及对应 SDK、Ninja；Python 建议 3.12 x64，包含 Tcl/Tk |
| 编译器选择 | 不限定 MinGW/MSYS2；MSVC 用开发者终端，GCC/Clang 用相应工具环境。原生 OpenCV 必须与核心的编译器、架构及运行库兼容 |
| Python 包 | `.venv/`，由 [pyproject.toml](pyproject.toml) 安装固定版本；不污染系统 Python，不提交虚拟环境 |
| 原生 SDK | 默认 `.deps/native/opencv/` 与 `.deps/native/deps/eigen-3.4.0/`；长期供本项目复用，不放 `outputs/`，不提交二进制 |
| 本机配置 | `.local/native.json` 由构建命令生成，记录核心库与 DLL 搜索目录；跨电脑或迁移目录后重建，不复制本机绝对路径 |
| 构建产物 | `build/python-core/bin/dt46_aim.dll` 等；可清理重建，不影响 `.deps/` 和 `.venv/` |
| 运行产物 | `outputs/`；环境准备、验收日志、录制、逐帧结果及截图均放这里 |
| 随 Git 交接 | 源码、模型、ROS YAML、依赖声明、CMake、准备脚本及测试；环境目录均由 `.gitignore` 排除 |

Python 的 `opencv-python-headless` 供外层绘图和视频使用，**不能代替 C++ 编译所需的原生 OpenCV SDK**。Tk 来自 Python 安装，不是 pip 包。`ctypes`、JSON、socket 为标准库，不增加 pybind11 或 Windows ROS 依赖。

## 开发环境工作流

在 DT46 仓库根目录运行；不需要把它嵌在仿真仓库中才能编译。先安装上述开发工具，选择工具链终端，确认 `python`、`cmake`、`ninja` 和编译器可用。

```powershell
# 使用项目内部 .venv；国内 pip 默认走清华源
./src/rm_vision_windows/setup.ps1 -Python python

# 已有兼容的原生 SDK 时显式复用，避免再下载和编译 OpenCV
./src/rm_vision_windows/setup.ps1 -Python python `
  -OpenCVDir '<OpenCV 安装目录>' -EigenDir '<Eigen 头文件根目录>'

# 更改 C++ 源码后重建核心；不需要重装 Python 包
./.venv/Scripts/dt46-build.exe
./.venv/Scripts/python.exe -m unittest discover -s src/rm_vision_windows/tests -v
./.venv/Scripts/python.exe src/rm_vision_windows/tests/source_audit.py --output outputs/source-audit.json
```

准备脚本执行：创建 `.venv` → pip 安装本包及依赖 → 缺少原生 SDK 时调用 [bootstrap.cmake](../../cmake/bootstrap.cmake) → 构建已有 C++ 核心 → 生成本机 `.local/native.json`。日志目录在完成时打印；任一步失败即停止。**准备完成只说明环境和编译完成，不能代替下文的接入验收。**

后续 `dt46-build` 默认复用 `.local/native.json` 中上次成功构建的 SDK、生成器、配置和显式运行库目录，命令行参数可以覆盖；首次构建默认使用项目内部 SDK。Windows SDK 的运行 DLL 和编译器运行库若不在标准路径，可给 setup 传 `-RuntimeDir @('<目录>')`，或给构建命令传 `--runtime-dir`。这里只记录路径，不把编译器复制入项目。切换编译器/架构/生成器前移走旧 `build/python-core/`，再重建兼容 SDK 和核心；不能混用先前缓存。

国内下载：pip 默认地址见脚本的 `IndexUrl`，可用 `-IndexUrl https://pypi.org/simple` 切回官方源；用法依据 [清华 PyPI 说明](https://mirrors.tuna.tsinghua.edu.cn/help/pypi/)。原生准备传 `DOWNLOAD_REGION=cn`，为 OpenCV 自身下载器设置 GitCode 镜像，依据 [OpenCV 官方 FAQ](https://github.com/opencv/opencv/wiki/FAQ)。当前 bootstrap 的主源码包和 FFmpeg 预取仍经过 GitHub/GitLab，不承诺全程国内加速；网络受限时优先复用团队提供的同版本、同工具链 SDK，并保留 bootstrap 中的校验值。只复制 `.deps` 的兼容原生安装，重新生成本机配置和虚拟环境。

## Windows 接入与 ROS 对应

界面与 C++ 版统一为 Windows 仿真调试四视图，顶部 `Theme` 可选 Day / Night（本版默认 Day），`UI font` 调整面板字号。配色、图像文字、保存帧、参数快照和 rqt 的边界统一见 [共用调试 UI 约定](../../standalone/README.md#虚拟相机与独立调试)。主题切换不重建 C++ 上下文；保存使用显示帧缓存，不读取之后生成的调试图或尚未应用的编辑值。

| aim_test 入口 / 数据 | Windows 外层对应 |
|---|---|
| Python launch 启动 C++ detector / tracker | `dt46-vision` 管理相机会话、C++ ABI 句柄及退出；仿真已有进程启动器可以直接拉起该 EXE |
| `/image_raw`、`/camera_info` | 相机协议的原始 RGB、K、D；算法前不翻转、不旋转；显示旋转只作用于窗口 |
| `/imu/rpy` | 每帧 `optical` 提供同步光学旋转，直接传给 C++；不从 RGB 猜姿态，也不额外叠加相机内参修正 |
| `/nav/decision` | 使用协议实际提供的目标颜色、实测弹速及物理设置；`follow_decision/target_color` 与核心配置对应。未提供的裁判字段不虚构 |
| `/tracker/gimbal_control` 的 yaw/pitch/can_fire | `gimbal_control` 结果保留原 EMA 输出字段；Windows `aim` 回传按现有协议使用同帧原始命令角，而非将旧帧平滑误差当作当前命令 |
| rqt 图像视图与参数编辑 | Tk 四视图、相机/检测器/跟踪器参数、保存/加载、日志、录制；图像来自同一 C++ 上下文 |
| ROS 节点重启 / 参数回调 | 参数校验后重建核心并递增 revision；失败保留旧上下文；仿真 generation 改变时重置 tracker |

线格式、坐标、单位和帧身份检查沿用 [共用通信与视觉契约](../../standalone/README.md)。Windows 的直接姿态输入和反馈包装是平台适配；没有据此宣称 DDS/QoS、消息时序或实车性能完全等价。

仿真项目已经有 `--vision-executable`，只替换启动参数，无需改仿真文件：

```powershell
& '<sim_vision.exe>' --root '<仿真仓库>' --headless-drive `
  --vision-executable '<DT46 仓库>/.venv/Scripts/dt46-vision.exe' `
  --output '<仿真仓库>/outputs/python-vision'

# 独立启动时连接仿真已产生的相机会话
./.venv/Scripts/dt46-vision.exe --root . --camera-session '<camera_session.json>' `
  --output outputs/vision --record outputs/vision/result.mp4
./.venv/Scripts/dt46-debug.exe --action show
./.venv/Scripts/dt46-debug.exe --action close
```

已有启动器传的 `--root` 是 `<仿真仓库>/DT46_V`，因此直接由它启动时，将此视觉检出放在该位置；这只是现有启动器的路径约定，**不是 C++ 编译依赖**。视觉放在其他目录时，用独立启动方式并传实际视觉根目录。调试会话默认 `.local/virtual_camera/session.json`，多实例用 `--debug-session` / `--session` 显式区分。调试命令返回 `requested` 只是排队成功，完成/失败查看 `debug_events.jsonl`，不能以排队成功当作调参成功。

## 验收及当前边界

```powershell
# 需要可用的仿真二进制、资源和其运行 DLL 环境；输出目录须不存在
./.venv/Scripts/python.exe src/rm_vision_windows/tests/acceptance.py `
  --simulator '<sim_vision.exe>' --sim-root '<仿真仓库>' --output outputs/windows-acceptance

# 现有仿真应用交互回归：第三个参数换为本包生成的可执行入口
& '<sim_application_check.exe>' '<仿真仓库>' '<sim_vision.exe>' `
  '<DT46 仓库>/.venv/Scripts/dt46-vision.exe'
```

本机验证为 Windows x64、Python 3.12.14、GCC 15.2、原生 OpenCV 4.12 / Eigen 3.4。这记录已测工具链，不限制团队必须选用该工具链。本轮 SDK 复用了兼容的预装版本，**未在一台干净电脑上从零下载重建全部 SDK，MSVC/Clang 未验证**。

| 实际检查 | 结果与证据 |
|---|---|
| DT46 脱离仿真源码独立构建 | 自有 `.deps` 编译 C++ 核心通过；完整原生构建 6/6 测试通过，含五个 tracker 基线文件和 ROS CameraInfo 兼容入口的 PnP 数值对照；[native-ctest-final.log](../../outputs/python_outer_20261026/native-ctest-final.log) |
| Python ABI / 无效配置 / 时间戳 / 分片通信 | 6 项测试通过，使用真实 C++ 库；[test_adapter.py](tests/test_adapter.py) |
| 原启动器自动拉起 | 278 帧，退出码均为 0；[外部仿真启动结果](../../../outputs/python_outer_20261026/startup/vision/report.json) |
| 既有仿真应用交互回归 | 驾驶、暂停/继续、复位、焦点释放、窗口开关、自瞄、发弹及断开通过；检查 674 帧、自瞄发弹 2 次、断开后 0 次；[原始 checks.json](../../../outputs/cpp_migration/workflow_41092/checks.json) |
| 可见 Tk / 调参 / 配置读写 / 录制 | 172 帧、56 个写入的视频帧全部解码；真实桌面截图，窗口控件回调实际执行，文件对话框路径由测试供给；[checks.json](../../outputs/python_outer_20261026/acceptance-final/checks.json) |
| 来源与 Python 语法 | [source-audit.json](../../outputs/python_outer_20261026/source-audit.json)，不替代 ROS 编译或运行 |
| ROS 2 / DDS / 实车相机、串口、IMU | **未验证**；本机没有 ROS 2/WSL 和对应硬件。没有把 Python 的 Windows 接通写成 ROS 实机迁移成功 |

首次可见窗口验收的句柄比较跨过了中间调参，导致测试误报，已修正比较区间；高 DPI 截图坐标亦经实际截图检查修正。首次记录保留在同级 `acceptance-first/`。`--debug-hidden` 产生的合成预览标记 `desktop_capture=false`，不拿它证明可见窗口正确。

## 依赖差别与移交视觉组

| 运行方式 | 保留的原生依赖 | 外层依赖 |
|---|---|---|
| 仿真平台 | MuJoCo、OpenCV、Eigen、nlohmann/json、GLFW、随源码提供的 TinyXML-2；版本由[仿真 lock](../../../cmake/dependency-lock.cmake)维护 | 正常仿真运行无 Python；离线工具单独处理 |
| DT46 纯 C++ 独立应用 | OpenCV、Eigen；原生外层另用 nlohmann/json、GLFW/OpenGL 与随 GLFW 源码提供的 Nuklear | 不需要 Python，见 [standalone](../../standalone/README.md) |
| 本包 Python 外层 + 原 C++ 核心 | OpenCV、Eigen；不构建原生 GUI/通信外层，因此核心本身不引入 GLFW、Nuklear、nlohmann/json | NumPy、PyYAML、Pillow、Python OpenCV，固定版本见 pyproject；Tk 随 Python |
| aim_test ROS 路径 | 原 C++ detector/tracker 及 OpenCV、Eigen、ROS 2 C++ 依赖；相机 SDK 按驱动选择 | ROS 2 Humble / rclpy / launch、消息包、colcon、pyserial；与 Windows pip 包分开 |

因此不能笼统说“纯 C++ 只多一个 YAML 库”，也不能说“有仿真后视觉不需要另外声明依赖”。当前 C++ 独立路径用 JSON 配置，不依赖 yaml-cpp；ROS 路径和 Python 外层的 YAML 读取另有各自实现。两边相同的原生 SDK 可以在兼容前提下复用，但必须由视觉自己的构建显式指向，不能偷偷从仿真源码或 Python wheel 取头文件。

采用本结构能省去视觉组理解一套替代 ROS 工程树的成本：仍在原包里改 C++ 算法，Linux ROS 的 Python glue 按原目录保留；在 Windows 只维护这个额外适配包。它省的是外层迁移，不会免除 C++ 工具链、OpenCV/Eigen 或实机回归。ROS 原 `rm_serial/package.xml` 未完整声明其源码实际导入的 rclpy、消息及 pyserial 依赖；本次保留来源文件不暗改，Linux 交接仍须核对 rosdep/包元数据并做 colcon 和硬件验收。
