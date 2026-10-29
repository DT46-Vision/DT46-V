# 视觉速度与弹道验证，2026-10-30

范围：视觉仓库 `codex/vision-speed-ballistics`，基线 `ee55828`。目标是降低每帧开销、删除重复计算，并验证在线弹道。父仿真源码和参数未修改；ROS、实车是后续阶段。本报告是实验快照，当前算法由 [rm_tracker](../../src/rm_tracker/README.md#直接弹道解算) 维护。

## 同图测速

AMD Ryzen 9 8945HX，Windows，MinGW GCC 15.2.0 Release，OpenCV 4.12.0、单线程。使用前轮保存的 9 张 1280×1024 RGB 图，覆盖近/中/远、静止/旋转/平移旋转。每张图分别测试固定 25 m/s、每帧交替 24.7/25.3 m/s；每个上下文预热 40 帧，再采 240 帧。两个 DLL 在同一进程中交错执行，每次交换先后次序。每种模式每版 2160 个样本。

计时覆盖 C ABI 的输入复制、检测、PnP、跟踪、弹道、调试 JSON 序列化；不含相机/传输、外层 JSON 构建、UI、日志写盘。输入哈希、二进制哈希和汇总见 [summary.json](summary.json)，脚本见 [benchmark.py](benchmark.py)。所有 18 组对照中的灯条数、板 ID、分类分数、角点和 PnP 坐标逐值相同。

| 弹速模式 | 旧中位数 ms | 新中位数 ms | 旧 P95 ms | 新 P95 ms | 旧最大 ms | 新最大 ms |
|---|---:|---:|---:|---:|---:|---:|
| 固定 | 2.128 | 1.800 | 2.521 | 2.020 | 2.901 | 3.026 |
| 每拍变化 | 7.502 | 1.828 | 7.995 | 2.167 | 11.856 | 2.906 |

固定弹速中位数下降约 15%，弹速变化下降约 76%；初始化中位数约 11.7→0.9 ms。Windows 单次最大值受调度影响，固定弹速的最大值没有改善，不能宣称消除了所有尖峰。前一轮交错测量得到 2.29/2.35 ms 的新版中位数，同样显示改进；最终表使用最后一次完整测量，不跨轮拼接新旧样本。

新版各帧分段中位数的中位数：准备 0.176 ms、检测 1.513 ms、PnP 0.018 ms、跟踪含弹道 0.013 ms、快照 0.074 ms。各段统计不应直接相加当作总时延分位数。剩余主要计算开销在检测。

## 弹道查证与选择

假设：旧表会因弹速变化重算而产生尖峰，粗插值和制表停止逻辑可能损害精度。检查发现每次弹速变化超过 0.1 m/s 都计算 25×15 个表格点，每点最多五轮 1 ms Euler 积分；最后一轮未收敛时还会更新俯仰角，但保留上一角度的飞行时间。角度/时间分别插值，适配器又重算一次弹道读取有效性。

方法：用独立时间域 RK4（0.1 ms）求实际弹道在目标水平距离处的高度和到达时间。网格为 20–30 m/s（步长 1）、水平距离 0.2、0.3、0.5–12 m（后者步长 0.25）、高差 −3.5–3.5 m（步长 0.25），平方阻力 0.019/m，共 15,631 点。零阻力另跑同一网格。预设替换标准为有阻力落点误差 <10 mm、时间误差 <0.6 ms；零阻力与精确抛体的高度差 <1 μm。

第一种沿目标视线固定有效阻力的直接近似最大误差 17.06 mm，未采用，保留 [formula_audit.py](formula_audit.py) 和 [结果](formula_audit.json)。最终方案先求无阻力低弧解，用其四分之一路程斜率近似有效阻力，再直接解二次式；该位置来自一阶阻力对下坠的权重 `(R-x)^2` 的重心。整个在线求解没有反复修正角度。

| 检查 | 旧表 | 新直接近似 |
|---|---:|---:|
| 全网格最大高度误差，仅统计到达距离的旧解 | 23.776 m | 4.838 mm |
| 全网格不向前/3 s 内未到达 | 1813 点 | 0 点 |
| 1–12 m、±0.5 m 高差最大高度误差 | 20.614 mm | 1.601 mm |
| 全网格最大到达时间误差，仅统计到达距离的旧解 | 2.766 s | 0.522 ms |

旧表最坏值来自近距离、大高差边界，不能据此说常见平射都错了。25 m/s、3 m 同高处，旧模型落点误差只有约 0.24 mm。当前仿真中低命中率仍可能来自观测、关联、更新周期和散布，不能归因于弹道一项。

31,262 点回归通过，含零阻力、无解、非有限输入、范围保护；独立测试见 [ballistics_test.log](ballistics_test.log)。一百万次变距离、变弹速调用，同时消费角度、时间和有效性，平均约 44 ns/次；这是计算微基准，不是完整视觉周期。有限网格只证明已测参数，不构成连续全域或实物精度保证。

## MuJoCo 的用途

实际运行 MuJoCo **3.13.0**：独立自由弹丸、RK4、1 ms 步长，通过被动力回调施加与仿真同形式的 `−k·|v|·v`。36 个速度/距离/高度组合与独立 RK4 的最大高度差为 **1.225 μm**，最大到达时间差约 0.068 μs。一次正向轨迹平均 **1.007 ms**，包含重置弹丸状态，不包含反求角度和整个机器人世界。源码及结果：[ballistics_audit.cpp](ballistics_audit.cpp)、[ballistics_audit.json](ballistics_audit.json)。

建议：在线用小型直接解算，MuJoCo 用作离线物理对照和仿真命中验证。官方 [mj_step](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-step) 推进动力学状态；它没有自动求“打到指定位置”的发射角度。其 [流体模型](https://mujoco.readthedocs.io/en/stable/computation/fluid.html) 也需要参数与几何假设。本次特意施加同一阻力方程，检验的是积分一致性，不是独立证明真实空气阻力系数。

本地父仓库 `project/modules/ballistics/physics.cpp`、`visual.cpp`、`runtime.cpp` 显示，现有弹丸使用独立分段常加速度积分，并非依靠 `mj_step` 推进飞行；出膛前后力模型不同。当前 shooter 的二次阻力为 0.019/m、线性阻力和 Magnus 为 0。在线解算按枪口后的重力/平方阻力建模。上述毫米误差相对高精度连续模型，不包含实际仿真 1 ms 步进误差、枪口测速误差或弹丸散布。

## 实际仿真周期和发弹

12 s 场景，实际 RGB→检测→PnP→跟踪→云台→弹丸→裁判扣血。表中“有效伤害弹”按裁判接受的唯一 shot_id 统计，不能使用碰撞事件总数充当命中弹数。四个最终 GUI 场景、两个无 GUI 移动旋转场景均通过：有发弹、有扣血、无无效开火许可、无窗口错误、进程正常退出。

| 运行 | 距离/运动 | 视觉处理中位数 ms | 新图间隔中位数 ms | 发射/有效伤害弹 |
|---|---|---:|---:|---:|
| GUI，纹理复用 | 3 m 静止 | 2.048 | 19.5 | 83 / 82 |
| GUI，纹理复用 | 5 m 静止 | 2.297 | 20.0 | 87 / 31 |
| GUI，纹理复用 | 3 m 旋转 | 2.404 | 19.5 | 54 / 49 |
| GUI，纹理复用 | 5 m 旋转 | 2.847 | 29.0 | 18 / 9 |
| 无 GUI | 3 m 平移＋旋转 | 3.412 | 11.0 | 64 / 30 |
| 无 GUI | 5 m 平移＋旋转 | 3.223 | 11.0 | 60 / 8 |

无 GUI 两组在纹理优化前运行，之后仅改显示纹理更新及错误文案。此前四个 GUI 场景新图间隔中位数为 31–40 ms；纹理复用后观察到 19.5–29 ms。两次闭环运行并非固定输入交错测试，受系统负载、轨迹和反馈影响，不能把周期差全部归于纹理分配。此处的新图间隔按仿真时间戳统计，采样请求上限为 100 Hz；`camera_ms=0` 没有分项测出相机耗时。仍不能把 1.8 ms 核心耗时说成整个系统每 1.8 ms 更新一次。

窗口截图已目视检查，四格、颜色、文字和板框正常；自动检查覆盖 RGB 与 BGR 同图等价、调试原图颜色、调用者改写输入后调试图仍保留原像素。原图对比不把参考绿框当作输入像素。5 m 平移旋转仍只有 8/60 个有效伤害弹，不能宣称稳定命中已完成。

## 变更与验证

运行时代码净减少 47 行；新增测试与实验档案单独计数。

- `ballistics.hpp`、tracker：直接近似替换粗表，角度与飞行时间一起返回；删除制表和双重查询。目的是稳定每拍成本并修复边界错误。独立 RK4、MuJoCo、31,262 点回归和实际发弹已验证。
- tracker/适配器：复用本帧弹道有效性，删除重复求解和重复清空缓存；最近板改为直接取最小距离，省去复制排序。车体/切板回归通过。
- tracker：后方或角度需要限幅的目标不再报告有效弹道，保持限幅转向输出；新增范围/后方/限幅检查通过。
- detector/原生入口：直接读取 RGB 通道，仍持有一份图像保障调试生命周期；灯条提取返回成员只读引用，省去无用向量复制。九图 × 两种弹速的检测、分数、角点、位姿完全一致；RGB/BGR 和调试图自动回归通过。
- 原生接口：新增五段只读计时接口，不改变 `AimResult` 布局。完整 C ABI 基准和原生管线检查通过。
- GUI：尺寸不变时复用纹理，普通刷新上传内容；四个带 GUI 的仿真场景及截图检查通过。
- ROS 适配器与离线回放调用：去除已删除的制表入口。原生回放工具已编译；ROS 编译与实车未验证，按用户要求留待后续。
- 文档：同步 tracker 算法、detector 输入通道和 standalone 接口，测量事实集中在本报告和 [summary.json](summary.json)。
- 默认启动文件：已更新 `build/native/bin/dt46_vision.exe`、`dt46_aim.dll`，旧版已备份，新旧路径与最终哈希见 [default-runtime.json](default-runtime.json)。

原生 CTest **12/12 通过**，见 [ctest.log](ctest.log)。本次先验证公式再替换；初版公式未达标准，采用直接计算的物理近似改进，没有接入在线迭代求解器。偏离原授权范围：无。剩余事项：较远移动旋转的稳定命中、完整周期中采图/传输/UI 的独立分项计时、未测模型参数范围；ROS 和实车按既定阶段后置。

## 复现

从视觉仓库根目录执行。Python 路径使用本机已配置的运行时，OpenCV/MuJoCo 路径来自父仓库现有环境。原始 RGB、逐帧日志、完整计时样本和二进制保留本机，不提交；首次复现同图基准需保留前轮 9 张原始快照。

```powershell
. ../.local/runtime-path-check/activate.ps1
$env:PATH=(Join-Path (Get-Location) 'build/aim-validation-full/standalone')+';'+$env:PATH
cmake --build build/aim-validation-full --target dt46_vision dt46_ballistics_check dt46_detector_check dt46_tracker_model_check dt46_tracker_switch_check dt46_pipeline_check --parallel 4
ctest --test-dir build/aim-validation-full --output-on-failure | Tee-Object outputs/vision_speed_20261030/ctest.log
& ./build/aim-validation-full/dt46_ballistics_check.exe | Tee-Object outputs/vision_speed_20261030/ballistics_test.log
& 'C:/Users/Unity/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' outputs/vision_speed_20261030/benchmark.py build/aim-validation-full/standalone/dt46_aim.dll outputs/vision_speed_20261030/comparison.json --baseline outputs/vision_speed_20261030/baseline-dt46_aim.dll
& D:/msys2/ucrt64/bin/g++.exe -std=c++20 -O3 -Isrc/rm_tracker/include -I../.deps/environment-path-check/src/mujoco-3.13.0/include outputs/vision_speed_20261030/ballistics_audit.cpp ../.deps/environment-path-check/src/mujoco-3.13.0/lib/mujoco.lib -o outputs/vision_speed_20261030/ballistics_audit.exe
& ./outputs/vision_speed_20261030/ballistics_audit.exe outputs/vision_speed_20261030
& ./outputs/vision_speed_20261030/run-cases.ps1 -Batch texture -Cases @('3_off','5_off','3_constant','5_constant') -Seconds 12 -Gui
& ./outputs/vision_speed_20261030/run-cases.ps1 -Batch motion -Cases @('3_constant','5_constant') -Translation constant -Seconds 12
& 'C:/Users/Unity/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe' outputs/vision_speed_20261030/summarize.py
```
