# aim_test 分支审查证据

本文保留指定提交的审阅快照，并集中维护四项问题的修复方案和验证记录。下方 R01 至 R04 的证据仍对应旧提交；当前实现进展见“修复方案与验证”。各模块使用说明继续由各包 README 维护。

## 审查范围

| 项目 | 内容 |
| --- | --- |
| 仓库 | `DT46-Vision/DT46-V` |
| 审查分支 | `aim_test` |
| 审查提交 | `3c41f0b02a05bf06ad28ca8440bc8099545498ab` |
| 对比基线 | 上次审查的 `89842af` |
| 审查日期 | 2026-10-22，Asia/Shanghai |
| 关注场景 | 5 m 以上检测框、角点或识别结果跳动；车辆轴心偏离地面法线不超过约 45°；新增小陀螺瞄准逻辑 |
| 方法 | 提交差异与调用链核对，摘取源码函数执行 C++ 合成反例 |
| 状态 | 旧版四项逻辑反例已复现，PR #3 已补充修复及工作树回归检查；现场主因、发生频率、ROS 2 集成和实车影响未验证 |

已确认代码存在下述问题，但没有实拍或 rosbag 证据将它们认定为现场远距离抖动的主因。检测框跳动直接对应 R01 的配对阶段；R02 至 R04 影响后续追踪或瞄准，不能用来解释上游检测框本身的跳动。

## 证据材料与复现

- [reproduce.py](reproduce.py)：默认通过 `git show` 读取审查提交；加 `--check-fixes` 时读取当前工作树，摘取相关逻辑并编译运行。
- [results.txt](results.txt)：此次验证的原始输出快照。`LIGHT`、`SPIN`、`YAW`、`HISTORY` 分别对应 R01、R02、R03、R04。
- [regression_cases.cpp](regression_cases.cpp)、[regression-results.txt](regression-results.txt)：修复回归输入和本次通过记录。
- 本文的仓库源码链接固定到审查提交，避免分支后续更新改变证据位置。

需要 Python 3、Git 和支持 C++17 的 `g++`，并保证审查提交存在于本地 Git 历史中。从仓库根目录运行：

```bash
python3 docs/reviews/aim-test-3c41f0b/reproduce.py
```

Windows 可用 `python` 运行同一脚本，编译器须在 PATH 中。脚本在系统临时目录生成 C++ 文件和可执行文件，运行结束清理，不覆盖结果快照。默认模式的退出码 0 表示旧提交的反例复现成功。

检查当前工作树修复：

```bash
python3 docs/reviews/aim-test-3c41f0b/reproduce.py --check-fixes
```

该模式的退出码 0 表示下述四组逻辑回归通过；它不能替代 ROS 2 或实车验收。合入原生视觉分支后，测试夹具提供与 `Tracker` 默认值相同的 `gravity=9.81`，供摘取的弹道函数使用。

### 验证边界

- 灯条输入是手工构造的端点，没有执行 OpenCV 轮廓提取、完整配对循环、分类模型或真实图片检测。
- 矩阵和向量使用只提供所需操作的简单替身；真实 Eigen、EKF 协方差、马氏距离和状态收敛没有运行。
- `MeasurementSink` 仅记录 `apply_armor_measurement` 传入的观测，不模拟真实 EKF 更新；不能由此推断滤波输出幅度或必然发生重置。
- 旧版开火反例摘取原地 SPIN 分支；修复回归摘取整个目标生成段、选板、弹道时间表和 `can_fire`，手工设定状态及枪口角误差。两者都没有执行完整 `track` 状态机、云台控制或实际发射。
- 历史过期反例固定 SPIN 为真，不提供新双板样本；完整识别链是否持续满足该条件尚未验证。
- 本机验证环境为 Windows、Python 3 和 MinGW g++；未进行 ROS 2 构建、相机测试、rosbag 回放或实车验收。

## 修复方案与验证

修复验证日期：2026-10-23（Asia/Shanghai）。

PR #3 的业务代码变更限定在检测器配对和追踪器中这四项逻辑；没有调整识别模型、标定参数、ROS 接口或倾斜车轴模型。

| 编号 | 具体修复 | 验证与预期行为 |
| --- | --- | --- |
| R01 | 在 `slopes_match` 中用 `abs(remainder(a-b, 360))` 计算最小角差，保持现有端点、PnP 点序和阈值 | 原 9.527° 反例通过；49 组倾角组合与几何期望一致，真实大夹角和过陡中心连线仍拒绝 |
| R02 | 原地和平移 SPIN 均复用现有四板到达时刻选板及车心正前瞄准。原地将线速度外推项置零；平移继续外推。瞄准点的半径和高度来自所选板；车心不可用时仅跟随观测、关闭 SPIN 发弹许可 | 原 5 m、4 rad/s 反例关闭许可；到达时正对的相邻板可获许可，交替半径/板高正确；顺逆转、±π 视线附近共 2160 个相位与独立四板窗口判据一致；平移提前量和非 SPIN 跟板保留 |
| R03 | 将 `best_match_yaw` 写回匹配观测副本，再传给换板和实际 EKF 更新；不修改输入消息 | 源码关联循环到更新入口的 12 组用例通过，包括镜像/非镜像以及 ±π 跨界；参考 +30° 的反例实际传入 +30°，原始输入不变 |
| R04 | 将 0.6 s 车心历史淘汰移入 `center_motion_is_slow`，以当前 `move_clock_` 为准，每次调用执行；移除仅在新双板出现时清理的代码 | 2 s 无新双板后旧样本清空并退出平移模式；单样本不恢复平移；新鲜连续平移可按原 0.45 s 滞回进入，原地和退出 SPIN 的回退通过 |

提交对应：R01 [54d5a2f](https://github.com/DT46-Vision/DT46-V/commit/54d5a2f)、R02 [48e1ab9](https://github.com/DT46-Vision/DT46-V/commit/48e1ab9)、R03 [1cc835c](https://github.com/DT46-Vision/DT46-V/commit/1cc835c)、R04 [9408747](https://github.com/DT46-Vision/DT46-V/commit/9408747)。

**R02 的行为选择：** 单独给当前板朝向加 `角速度 × 时间`，仍会让瞄准点停留在旧板位置，不能解决时刻不一致。因此本次原地 SPIN 改为复用已有的车心正前等板路径，选板使用 `飞行时间 + system_delay`，`controller_delay` 仍只影响平移瞄准点。该修复重新依赖车心估计的稳定性，需要用原地小陀螺回放检查红圈抖动及模式切换；当前回归只证明理想输入下的几何与逻辑一致。

**实现入口：** [检测器](../../../src/rm_detector/src/detector.cpp)的 `is_armor`；[追踪器](../../../src/rm_tracker/src/tracker.cpp)的 `update`、`center_motion_is_slow` 和 `track`。当前运行行为分别维护在 [rm_detector 处理流程](../../../src/rm_detector/README.md#处理流程)和 [rm_tracker 数据流与算法](../../../src/rm_tracker/README.md#数据流与算法)。

**尚待目标环境验收：** 在 Ubuntu 22.04 / ROS 2 Humble 中先构建 `rm_interfaces`，再以 `colcon build --packages-select rm_detector rm_tracker` 构建相关包；回放远距离检测、原地/平移旋转、双板转单板序列，检查真实 EKF 状态、观测时间及发弹窗口；最后验证实车延迟和弹着点。当前环境没有可用的 ROS 2 构建链，以上未验证。

## R01 · P1 · 灯条角差未处理 ±180° 环绕

**源码依据：** [端点构造](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_detector/src/detector.cpp#L208)将 `box[0]`、`box[3]` 的中点命名为 `up`，将 `box[1]`、`box[2]` 的中点命名为 `down`。[OpenCV RotatedRect::points 接口说明](https://docs.opencv.org/4.13.0/db/dd6/classcv_1_1RotatedRect.html)规定顶点顺序为 bottomLeft、topLeft、topRight、bottomRight，因此近竖直灯条的 `down-up` 朝向图像上方，`atan2(dx,dy)` 位于 ±180° 附近。[斜率筛选](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_detector/src/detector.cpp#L361)直接相减两个角度，没有计算最小角差。

**输入：** A 的端点为 `(100,106)→(101,94)`，B 为 `(128,106)→(127,94)`，单位 px；两灯中心等高，允许夹角为 35°。对照输入将 B 改为 `(128,106)→(129,94)`。

**预期与实际：** 原输入的几何夹角约 9.527°，应通过这一角度条件；实际计算为 350.473°，`slopes_match=false`。对照输入则为 `true`，确认错误发生在角度环绕处。

**结论与影响：** 该输入会被错误跳过，轻微端点变化可能造成装甲配对时有时无。反例证明配对条件错误，未证明实拍图像会以多高频率产生这些端点。

**来源：** `46a88a2` 的灯条配对改动，由 `aim_test` 继承。

**后续实测：** 通过远距离原始图像确认误拒是否下降；代码修复和逻辑验证见上方方案表 R01。

## R02 · P1 · 原地旋转开火条件没有使用到达时刻

**源码依据：** [原地 SPIN 瞄准分支](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/tracker.cpp#L1153)计算了飞行时间，但 `spin_` 为真时仍瞄准当前 `plate_pos`，并以当前 `last_plate_yaw_` 计算正对误差。[开火条件](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/tracker.cpp#L927)直接比较这个当前误差与半宽窗口。

**输入：** 手工设定原地 SPIN，当前板位 `(5,0,0)` m，半径 0.24 m，当前朝向 0，恒定角速度 4 rad/s，四板相位间隔 90°。传给 `can_fire` 的 yaw/pitch 误差为 0、弹道有效标志为真。弹速 32 m/s、阻力系数 0.019、系统延迟 0.1 s 和可射距离 10 m 对应[哨兵配置](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/config/tracker_params_sentry.yaml#L55)。

**预期与实际：** 用源码弹道表得到飞行时间加系统延迟为 0.2646 s。到达时转过约 60.642°，最近板偏离正对方向约 29.358°，按程序所用弧长判据得到 0.122975 m，超过其 0.059 m 窗口；当前 `can_fire` 却返回真。此计算未加入额外 0.02 s 控制器延迟。

**结论与影响：** 在上述理想恒速条件下，当前许可与到达时刻的几何窗口不一致。它是开火时序逻辑的反例，不是实际弹着点或实车漏弹率的测量。

**来源：** `2e1a049` 新增的原地 SPIN 跟板分支。

**后续实测：** 用同步时钟下的旋转目标回放核对许可窗口，再做实际控制链验证；本次策略选择和逻辑验证见上方方案表 R02。

## R03 · P2 · 消歧后的 yaw 没有传入观测更新

**源码依据：** [匹配阶段](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/tracker.cpp#L694)计算 `best_match_yaw` 并据此判断匹配角差，但保存到 `best_match` 的仍为原始 `armor`。[更新调用](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/tracker.cpp#L720)将它传给 [apply_armor_measurement](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/tracker.cpp#L655)，后者重新读取原始 `armor.yaw`。

**输入：** 预测朝向 +30°，原始观测 −30°，板位 `(5,0,0)` m，参考车心为板位加上长度 0.24 m、方向 +30° 的水平向量。

**预期与实际：** 消歧选择 +30° 后，匹配和观测更新应使用同一结果。实际消歧结果为 +30°、匹配角差为 0°，观测记录替身收到的仍为 −30°。

**结论与影响：** 已证明消歧结果没有贯穿更新，匹配门和滤波观测使用了不同朝向。真实 EKF 随后拉动状态还是拒收，以及影响幅度，均未验证。

**来源：** 消歧逻辑来自 `2649f52`；`46a88a2` 提取 `apply_armor_measurement` 后仍保留该问题，均由 `aim_test` 继承。

**后续实测：** 检查进入真实 EKF 的观测，并比较镜像解交替时的状态稳定性；代码修复见上方方案表 R03。

## R04 · P2 · 双板观测历史不会按当前时刻过期

**源码依据：** [center_motion_is_slow](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/tracker.cpp#L259)仅检查历史样本之间的时间差，没有检查最新样本相对 `move_clock_` 的年龄。[历史清理](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/tracker.cpp#L317)处于获得至少两块板的分支内。[瞄准模式切换](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/tracker.cpp#L423)继续使用该判断。

**输入：** 两个车心样本为 `t=0 s: (5,0) m`、`t=0.5 s: (5,0.4) m`。保持 SPIN，不再追加样本，每次推进 0.01 s，共 2 s；重复调用瞄准模式更新。

**预期与实际：** 观测不足时应按代码注释回退，而不应把陈旧样本当作持续运动证据。实际最新观测已过期 2 s，`center_motion_is_slow=false`，`spin_aim_translate=true`。

**结论与影响：** 在持续 SPIN、无新双板的条件下，旧的平移判断不会自行失效，并能推动瞄准模式切换。是否在真实检测序列中持续满足该条件尚未验证。

**来源：** `2e1a049` 将运动判断改为双板历史；`3c41f0b` 调整位移门限，没有补充过期判断。

**后续实测：** 回放双板变单板、重新出现双板和退出 SPIN 的序列；代码修复与合成验证见上方方案表 R04。

## 倾斜场景的已有证据

以下是本次关注场景的静态限制，不作为新增的第五项回归问题：

| 证据 | 能支持的结论 | 尚不能支持的结论 |
| --- | --- | --- |
| [检测配置限制灯条角度为 ±35°](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_detector/config/detector_params_sentry.yaml#L8)，[检测器执行筛选](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_detector/src/detector.cpp#L201) | 图像中灯条倾角超限会被排除 | 车轴倾角与图像灯条倾角不能直接画等号，不能据此声称车轴超过 35° 必然漏检 |
| [消息新增 nx/ny/nz](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_interfaces/msg/ArmorInfo.msg#L10)，[追踪入口仍只转换位置和 yaw](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/node.cpp#L218) | 法向没有用于这里的姿态转换 | 不能把消息字段的增加视为已支持倾斜车轴 |
| [位置预测采用水平圆周](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/tracker.cpp#L30)，[其他板使用交替固定高度](https://github.com/DT46-Vision/DT46-V/blob/3c41f0b02a05bf06ad28ca8440bc8099545498ab/src/rm_tracker/src/tracker.cpp#L709) | 当前模型没有表示任意倾斜的旋转轴 | 不能保证轴心偏离地面法线不超过 45° 的全部姿态都能正常追踪；静止倾斜板仍可能被检测 |

## 尚缺的现场证据

1. 实际运行提交、相机型号、内参、分辨率及曝光设置。
2. 原始视频或 rosbag，以及同时间戳的灯条端点、配对结果和识别 ID，用于确认 R01 是否对应现场检测跳动。
3. PnP 输出、进入 EKF 的观测、追踪状态、双板样本时间和瞄准模式，用于检查 R03、R04 的实际触发。
4. 已知转速与时间延迟的旋转目标数据，以及云台姿态和开火许可，用于验证 R02。
5. 已知车轴倾角、距离和观察方向的对照数据，用于量化倾斜识别与追踪成功率。

本 PR 交付审查证据、四项逻辑修复及回归材料。ROS 2 集成验证和实车验收尚未完成，逻辑回归通过不代表这些验收已经通过。
