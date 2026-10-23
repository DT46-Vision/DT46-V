# 原生视觉功能与原始 tracker 审查快照

日期：2026-10-23，Asia/Shanghai。审查起始提交为个人分支 `codex/sim-vision-work@70447fee6fab2b62c90990f79efbd54d79e908b2`；tracker 对照用户明确指定的 GlennAde `3c41f0b02a05bf06ad28ca8440bc8099545498ab`。本目录记录本次审查与恢复后的验证，不是运行契约的第二维护入口；当前契约见 [standalone](../../../standalone/README.md)。机器可读证据摘要见 [checks.json](checks.json)。

## 结论与范围

原始版本的 detector、分类、PnP、tracker 和 EKF 已经是 C++，不存在本次把 Python tracker 翻译成 C++ 的过程。原提交的 31 个 Python 文件包括 17 个启动入口，以及串口、IMU、录制、YAML 变体、包安装和空模块文件。

Windows 原生链路、三种配置回放、录像和原始 tracker 基线检查通过。本次发现并补齐了原生适配层漏做的目标颜色过滤。tracker 核心已恢复指定提交；原 ROS 节点和 EKF 原本就与该提交一致。

**不能认定“原 Python ROS 版本所有功能已完成运行等价验收”**：ROS 2 节点、实物相机、MCU 和 IMU 未在本机编译或接硬件运行；串口调度与部分可配置能力也存在下述差异。源码对应、协议测试和 Windows 测试不能替代这些验证。

## Git 查到了哪些 tracker 改动

以下是审查起始提交相对原始起点的历史，不是本次新发明的逻辑。

| 历史提交 | 当时的改动 | 本次处置 |
|---|---|---|
| `c1c4b41` | 删除水平距离小于 1.5 m 的直接几何瞄准分支，近距离也走弹道表；固定重力改成成员参数 | 恢复原分支和固定重力 |
| `1cc835c` | 将消歧后的 yaw 写回观测，再进入 EKF 更新 | 撤回，保留原观测更新方式 |
| `9408747` | 每次平移判断按当前时间淘汰过期双板历史 | 撤回，保留原清理时机 |
| `48e1ab9` | 原地 SPIN 也改为到达时刻选板、车心正前等板；改变回退时的发弹许可 | 撤回，保留原地跟当前板、平移正前等板及原许可 |
| `e2590d6` | 在 tracker 内添加本帧 `aim_valid` 状态 | 撤回，在原生适配层导出有效性 |
| `1f11a90` | `RmTF` 增加直接光学旋转输入和保留标定主点选项 | 保留 Windows 相机适配；ROS 默认路径不变 |

恢复后的 `tracker.cpp`、`tracker.hpp` 与原 Git 内容一致；`ekf.cpp`、`ekf.hpp`、ROS `node.cpp` 经归一化换行后也一致。CTest 对这五个文件执行 SHA-256 校验。坐标适配文件 `rm_tf_tools.*` 不在这五个文件中，不能称整个 `rm_tracker` 包没有差异。

检测器仍保留历史提交 `54d5a2f` 的一行角度环绕修复：灯条角差由 `abs(a-b)` 改为 `abs(remainder(a-b,360))`。本次未改动该项；因此整套视觉算法不等于逐文件恢复到原提交。PnP 接口去除 ROS 消息依赖、改收普通标定结构，保留 IPPE 求解路径。旧审查对三个 tracker 问题的反例仍作为历史记录保留，当前分支按用户要求采用原行为，不能再把旧修复回归当作当前必过检查。

## 逻辑改动子分支与个人分支清单

按用户后续要求，新建本地分支 `codex/sim-vision-tracker-changes`，起点为恢复前的个人主线 `70447fe`，说明提交为 `032dfb1`。它完整保留恢复前的代码，不是重新实现或移植一组算法补丁。与 `70447fe` 的整棵 Git 树比较，仅 `standalone/README.md` 不同；该页记录与原始 tracker / 恢复后 `5923209` 的行为差异及当时尚未补齐的颜色过滤。本节上表已经列出各项逻辑来源，不另维护一份算法说明。

```powershell
git show codex/sim-vision-tracker-changes:standalone/README.md
git diff 5923209 codex/sim-vision-tracker-changes -- src/rm_tracker/src/tracker.cpp src/rm_tracker/include/rm_tracker/tracker.hpp standalone/aim.cpp
```

以下为 2026-10-23 本工作区个人开发所涉及的分支快照。DT46-V 远端分支名及提交通过 `git ls-remote --heads origin` 核对，并通过 GitHub API 核对 14 条远端分支头的作者账号；当前登录为 `unity-as`，远端三条 `codex/` 分支头均为该账号。未推送、删除、改名或重置已有分支。祖先关系只表示提交历史已在主线中，不表示后来没有撤回其行为。

| DT46-V 个人分支 | 本地位置 / 远端位置 | 相对个人主线的情况 |
|---|---|---|
| `codex/sim-vision-work` | 本地以 `5923209` 为本次代码比较点；远端 `70447fe` | 当前主线，保留原 tracker；本次登记另增加文档提交 |
| `codex/sim-vision-tracker-changes` | 本地 `032dfb1`；未推送 | 新保留的 tracker 改动版，代码等同 `70447fe` |
| `codex/aim-test-sim` | 本地 `b412a22`；无同名远端 | 早期虚拟相机接入，分支头已是主线祖先 |
| `codex/cpp-migration` | 本地 `6cc4492`；远端 `5ba00e2` | 原生迁移，分支头已是主线祖先；本地比该远端多 2 个提交 |
| `codex/project-branch-isolation` | 本地 `4e6c93f`；无同名远端 | 分支隔离规范入口，分支头已是主线祖先 |
| `codex/docs-aim-test-review` | 仅远端 `3d5bbb6` | 原四项审查/修复；提交历史已在主线中，其中 tracker 改动在 `5923209` 撤回 |

`2027` 本地分支头为 GlennAde 的 `a49f697`，不列为本次个人开发分支。其余远端分支头的作者：`2025` / `2026` 为 `Kielas520`；`2027` / `CarMidPoint-armor_try` / `Tracker_to_C++` / `aim_test` / `cuda` / `stable` 为 `GlennAde`；`feat/ballistic-ls-fit` 为 `wren-or-avenir`；`nav_interface` 为 `wwweng6216`；`rm_watch_dog` 为 `xkh-829`。作者账号本身不等于分支所有权，本清单结合本会话已确认的个人开发历史划定范围，没有扫描账号下其他仓库，也没有改动上述分支。

sim_vision 是用户的独立个人项目，本地共 18 条分支，无远端。除当前分支和同头的隔离分支外，其余分支头均是当前 `98d6b54` 的祖先；本次只列出，没有清理或修改这些分支。

| sim_vision 分支 | 分支头 | 内容 |
|---|---|---|
| `codex/sim-vision-work` | `98d6b54` | 当前个人工作分支 |
| `codex/project-branch-isolation` | `98d6b54` | 与当前工作分支同头 |
| `codex/camera-presets` | `649755a` | 相机预设、海康单位适配 |
| `codex/hikrobot-camera-reference` | `7cae7fe` | 海康资料与预设依据 |
| `codex/camera-sampling-sensor` | `b18e8b1` | 曝光响应、帧参数快照、采样限速 |
| `codex/indoor-shadows` | `e5f0b44` | 阵营色、泛光、曝光采样 |
| `codex/original-infantry` | `cac0f93` | 老步兵与原驾驶路径 |
| `codex/cpp-migration` | `ad04e2f` | 原生迁移及照明研究阶段 |
| `main` | `ad04e2f` | 与迁移分支同头，旧整合位置 |
| `codex/filament-renderer` | `7b18662` | Filament GPU 畸变和异步交付 |
| `codex/fix-render-lod-correctness` | `16353e5` | CAD 命中隔离、减面距离与缓存 |
| `codex/optional-shadows` | `57ee4fb` | 简化照明、可选投影阴影 |
| `codex/profile-camera-pipeline` | `e09f607` | 不限频采图与阶段计时 |
| `codex/optimize-camera-transfer` | `bcd8c45` | RGB 复制重叠、截止时间交付 |
| `opt/gpu-camera-pipeline` | `ab28460` | GPU 畸变、推进与采图解耦 |
| `opt/realtime-camera-benchmark` | `908982a` | 原始标定分辨率高频采图基准 |
| `opt/render-lod` | `636632f` | 减面入库与分段帧率口径 |
| `opt/visibility-culling` | `c4dbdd7` | 视锥和阴影贡献剔除 |

本次子分支保存只改说明文档，使用 Git 树差异、分支祖先关系和主线五文件原始基线核对，不重复执行运行测试；此前验证仍按本报告原有范围解释。偏离：无。未完成：没有新增未完成事项；此前 ROS 与实机缺口继续保留。

## 功能对照

| 原功能 | 当前对应 | 本次证据与边界 |
|---|---|---|
| 17 个 Python 启动入口 | 同目录 XML，包括整机、nav、bag、单节点、RViz、三种相机 | XML 解析、局部 include 路径、实际启用的字面量节点包名/可执行名清单通过；没有执行 ROS launch，也没有证明所有替换表达式运行正确 |
| 检测、ONNX 分类、原始畸变图 PnP | 原 C++ 核心与原生管线 | 实际渲染图回放；PnP 中心/边缘畸变与光学旋转测试通过。没有整图反畸变节点，调试图仍按原链路显示原图及叠加 |
| tracker、EKF、选板、弹道与许可 | 原始核心加输入/输出适配 | 五文件基线检查、回放、Windows 实际自瞄与发弹；不是命中率测试 |
| MCU Python 串口和 CRC | C++ 硬件协议库与 `rm_serial` ROS 节点 | 默认 A5/5A 帧的编码、解码、CRC、噪声/拆包/坏包检查通过；ROS 节点、端口时序与断线重连未验证 |
| Python 达妙 IMU 解析和发布 | C++ 协议库与 `dm_imu` ROS 节点 | 原特殊 CRC 更新方式、两种校验范围、RPY/四元数离线检查通过；实际设备、QoS 连接、消息频率未验证 |
| 迈德威视、海康、USB 相机 ROS 驱动 | 保留各 C++ 驱动及 XML 入口 | 本次静态检查启动关系；未接相机。Windows 虚拟相机不是这些 SDK 驱动的 Windows 移植 |
| 原 rqt 已保存布局 | 原生四格图、相机/检测器/tracker 三页、日志及参数文件 | 本次打开/关闭/重开同一识别上下文通过；布局和参数细节以 standalone 为准。ROS 参数服务器、点击话题和其他 rqt 插件不由原生页实现 |
| Python tracker MP4 录制 | 原生录像、ROS `record_images` 入口 | 三种预设各写入 4 帧 MP4；媒体测试另做 20 帧写入/读回。ROS 图像订阅录制未验证 |
| Python 哨兵 YAML 变体工具 | `dt46_sentry_variant` / ROS 安装入口 | 用原哨兵 YAML 实际生成四参数变体，数值为 20、0.01、100、80，其余正文和注释不变；ROS 安装入口未验证 |
| 标定、图像导出与离线回放 | 原生工具及 ROS 工具包装 | 20 视图已知参数标定和回放通过；实机标定精度、ROS 图像采集未验证 |

原 `bag_record` 的 tracker 节点已被注释，XML 沿用检测器加 rqt，并非迁移丢失 tracker。原 `mv_nav` 未启动 IMU，当前也未新增；缺少姿态发布时不能期待原 tracker 正常解算。这些属于继承的启动条件，没有替用户改掉。

MCU 默认字节协议一致不代表调度完全一致：原 Python 使用接收/发送两条线程；C++ 是单个 I/O 线程，写阻塞会影响接收时机。默认发送间隔为 5 ms，实际还受 I/O 和调度限制；没有实测串口吞吐或压力场景。原 Python 可自定义帧头，当前实现只接受 A5/5A；`pub_rpy` 在原代码中被强制关闭，当前会遵守配置。具体参数由 [串口说明](../../../src/rm_serial/README.md)维护。这些都是严格等价判断的边界，本次没有擅自改协议或重构调度。

Windows 相机采集与视觉是独立进程，但原生视觉内部取图、检测、tracker 和调试图生成按会话循环执行。原 ROS detector/tracker 各有工作线程，调度结构并不相同；不能从核心源码一致推断两种运行方式的速率或延迟相同。

## 本次具体修改

| 位置 | 改了什么、原因与影响 | 验证 |
|---|---|---|
| `src/rm_tracker/src/tracker.cpp`、`include/rm_tracker/tracker.hpp` | 恢复指定 Git 内容，撤回上表的 tracker 算法与有效性状态改动；恢复原有行为，包括旧审查指出的行为 | 基线校验与运行检查通过 |
| `standalone/aim.cpp` 颜色输入 | 在送入 tracker 前增加与原 ROS 节点相同的按 ID 过滤；检测页选两色或手动检测其他颜色时，仍只追踪目标颜色 | 真实渲染图反例修复前后对照通过，见下表 |
| `standalone/aim.cpp` 有效性输出 | 清空只供渲染的目标缓存，调用原始无状态弹道函数读取本帧有效性；不向原 tracker 插入状态。每个有目标的帧多一次原查表/三角计算，未单独测量其耗时 | 提前退出、复位及实际控制流程通过；没有据此声称开销为零 |
| `standalone/aim.cpp` 重力输入 | 固定导出原核心的 9.81，拒绝不同重力配置，避免配置显示已生效而算法未跟随 | 非法重力拒绝且上下文/revision 保持不变的测试通过 |
| `tests/original_tracker_check.cmake`、CTest、管线测试 | 增加五文件基线保护及重力配置拒绝检查；基线测试连注释变更也会拒绝，允许换行差异 | 6/6 测试通过 |
| 包 README、standalone、旧审查入口、本目录 | 运行说明改回实际原算法；旧修复结果标明历史边界；记录当前证据和未验证项 | 编码、链接及源码对照检查；不改写旧结果快照 |

## 验证结果

假设与方法：以 Git 内容校验“核心不改”；以目标色与检测色分离的渲染图回放检查 ROS 颜色门是否遗漏；以三种原 YAML/模型预设和原生 CTest 检查接口、媒体及几何；以 Windows 实际窗口操作流程检查驾驶、调试、自瞄与发弹。若出现非目标颜色追踪、旧帧有效性、断开后发弹或基线哈希变化，则相应检查不通过。硬件和 ROS 缺口单列，不以本机测试代替。

最终原生构建成功，CTest **6/6 通过**：`native_media`、`original_tracker_baseline`、`hardware_protocol`、`native_pipeline`、`native_transport`、`calibrated_pnp_and_optical_rotation`。仿真侧关联的 `auto_aim_control`、`camera_sampling` **2/2 通过**。

颜色反例使用同一段实际渲染的蓝方 ID 5 图像，共 20 帧；只把检测阈值设为 40 以稳定检测，不向视觉提供对手位置或编号真值。

| 参数组合 | 恢复前检测帧 / TRACKING 帧 | 恢复及补齐颜色门后 |
|---|---|---|
| 目标蓝，检测跟随目标色 | 20 / 15 | 20 / 15 |
| 手动目标红，检测两色 | 20 / **15（错误追踪蓝方）** | 20 / **0** |
| 手动目标红，检测蓝色 | 20 / **15（错误追踪蓝方）** | 20 / **0** |
| 手动目标红，检测跟随目标色 | 0 / 0 | 0 / 0 |

`infantry`、`sentry`、`hero` 三个预设在同一 20 帧图像集上均完成 20 帧检测、15 帧 TRACKING、4 帧 MP4 输出。这证明预设可加载并运行，不证明每种实车的识别精度或旋转目标性能。媒体测试另覆盖 20 视图合成标定和 20 帧视频回读；合成误差不等于真实镜头误差。

恢复后的 Windows 完整操作流程 `workflow_45276` 通过，共核对 1,138 个视觉帧：驾驶、暂停/继续、复位、失焦释放、画质保存、同一上下文调试窗口开关、右键真实图像自瞄、右键加左键许可发弹 2 次、松左键停火、断开视觉后无追加发弹且可继续驾驶、松右键恢复手动、正常关闭。该运行相机目标/有效上限为 300 Hz，交付约 281.17 Hz；包含窗口操作与暂停，不是不限速吞吐基准，不能写成相机最高性能。

失败记录没有删除：恢复前第一次 Windows 流程 `workflow_27924` 在画质面板操作后等待 `rendering.json` 超时，此前步骤通过。具体失败原因未确认；恢复后的新流程通过不能解释或抹去这次失败。另一次离线回放因命令未设置运行时 DLL 的 PATH，以 `0xC0000135` 在程序启动前退出，补齐依赖路径后才执行完成；不能算成算法失败或一次通过。

## 复现与证据位置

先按 [构建说明](../../../standalone/README.md#构建)准备依赖 PATH。以下原生命令从 DT46-V 根目录运行：

```powershell
cmake --build --preset release --parallel 4
ctest --preset release
git diff 3c41f0b -- src/rm_tracker/src/tracker.cpp src/rm_tracker/include/rm_tracker/tracker.hpp src/rm_tracker/src/ekf.cpp src/rm_tracker/include/rm_tracker/ekf.hpp src/rm_tracker/src/node.cpp
```

最后一条应无差异。完整 Windows 流程从 sim_vision 根目录运行：

```powershell
ctest --preset release -R 'auto_aim_control|camera_sampling'
build/release/sim_application_check.exe D:/project/RobotMaster/sim_vision D:/project/RobotMaster/sim_vision/build/release/sim_vision.exe D:/project/RobotMaster/sim_vision/DT46_V/build/native/bin/dt46_vision.exe
build/release/sim_sequence.exe --seconds 0.2 --camera-hz 100 --width 640 --height 512 --quality low --output outputs/native_parity_20261023/sequence
```

该序列按 100 Hz 的仿真时间戳离线生成，不测量实时采样速度。在结果目录分别准备参数 JSON：默认 `{"detector":{"binary_val":40}}`；颜色反例另设 `detector.detect_color=2` 或 `1`，同时设 `tracker.follow_decision=false`、`tracker.target_color=0`。从 DT46-V 根目录分别回放；三预设录像则保持默认参数并切换 `--profile`：

```powershell
build/native/bin/dt46_replay.exe --sequence ../outputs/native_parity_20261023/sequence/frames.jsonl --output ../outputs/native_parity_20261023/replay --parameters <参数.json> --dump-debug
build/native/bin/dt46_replay.exe --profile infantry --sequence ../outputs/native_parity_20261023/sequence/frames.jsonl --output ../outputs/native_parity_20261023/profile_infantry --parameters <参数.json> --record ../outputs/native_parity_20261023/profile_infantry/tracking.mp4 --dump-debug
```

本机原始图像、逐帧日志、参数和汇总保留在 [native_parity 输出](../../../../outputs/native_parity_20261023/)及 [Windows 流程结果](../../../../outputs/cpp_migration/workflow_45276/checks.json)，不纳入 Git。可移交的摘要在本目录 `checks.json`；不能把本机路径存在视为其他机器已拥有这些素材。本次一次性检查均通过标准输入执行，没有新建临时脚本；新增 CMake 测试是需要保留的正式检查。

偏离：原审查中不再将 R02/R03/R04 的修复回归设为当前验收标准，改用用户明确指定的原始 tracker 基线；本次未改仿真源码，也未向其他分支合并或推送。

未完成：ROS 2 Humble 的按包编译、启动/参数/话题运行检查；三种物理相机、MCU、IMU 接线测试；串口压力/重连时序、原地与平移 SPIN 的实车回放及命中精度。对这些项目目前无法得出运行等价结论。
