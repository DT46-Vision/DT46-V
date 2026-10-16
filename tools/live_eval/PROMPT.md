# 新分支开工提示词：DT46-V 视觉自瞄（tracker 位置 / PnP / 验证口径）

> 用法：把本文件整段贴给新的 AI 会话（或作为 Cursor 的项目规则）。它包含**已用实车数据核实过的结论**＋**待修清单**＋**必须照做的验证口径**＋**别重复踩的假线索**。
> 生成时间：2026-10-16 晚，基于当时仓库状态与三轮实车订阅数据。

---

## 0. 角色与硬约束

你是 ROS 2 / C++ 视觉自瞄工程师。工作区 `/home/glennade/DT46-V`（ROS 2 Humble / Ubuntu 22.04）。
仓库约定（见根目录 AGENTS.md）：**最小改动、不改无关代码、不加没要求的功能/可配置项、匹配现有风格、中文 commit、不确定先问不要猜**。

本轮目标：在**新分支**上按下面 P0→P7 逐条修 tracker / detector，**每条都要用第 4 节的验证口径实测**，通过才提交。

---

## 1. 环境 / 构建 / 运行

```bash
cd /home/glennade/DT46-V
# 新分支（三选一，见 §6）
git stash push -m "wip-before-new-branch-$(date +%m%d_%H%M)"   # 先把当前脏改动存起来（不要用 -u/-a，build/ 是 ignore 的）
git switch -c fix/tracker-position-$(date +%m%d) HEAD

source /opt/ros/humble/setup.bash
colcon build --packages-select rm_interfaces rm_detector rm_tracker --symlink-install \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ros2 launch rm_vision_bringup sentry.launch.py      # 哨兵；hero/infantry 见 launch/ 目录
```

关键话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/detector/armors_info` | `rm_interfaces/ArmorsMsg` | `armor_id`、`dx/dy/dz`(mm,相机系)、`yaw`(deg,相机系板 yaw) |
| `/detector/armors_debug_info` | `ArmorsDebugMsg` | 四角点像素（l1u/l1d/l2u/l2d），离线复算 PnP 用 |
| `/imu/rpy` | `geometry_msgs/Vector3Stamped` | 100Hz，**原始** yaw（tracker 内部会做开机归零，见 F10） |
| `/tracker/gimbal_control` | `GimbalControl` | `yaw/pitch`(deg, 相机系增量)、`can_fire` |
| `/nav/decision` | `Decision` | 下位机上行：敌人颜色 `color`、`bullet_speed` |

**采集探针（必须在沙箱/容器里加 UDP 环境变量，否则 0 消息）**：

```bash
export ROS_LOG_DIR=/home/glennade/DT46-V/build/roslog          # rclpy 要写日志，只读 home 会失败
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4                         # 共享内存被沙箱挡了，不加订阅不到数据
source /opt/ros/humble/setup.bash && source install/setup.bash
cd build/live_eval
python3 live_probe.py  40 <前缀>    # 装甲/IMU/云台 → <前缀>armors.csv / imu.csv / gimbal.csv
python3 debug_probe.py 40 <前缀>    # 四角点 → <前缀>_debug.csv
```

---

## 2. 已核实的硬事实（**不要重新推导、也不要推翻**，都有数据/脚本支撑）

- **F1 detector 的 id 约定**（`rm_detector/src/detector.cpp: get_id()`）：**红板 → 6/8/11，蓝板 → 0/2/5**。
  tracker 的 `target_color`：0 = 收 ≥6（红），1 = 收 <6（蓝），代码与 detector 自洽；`tracker_params_*.yaml` 的 `# 0: RED, 1: BLUE` 注释**是对的**（已核对），**只有 `rm_tracker/src/node.cpp` 里那句 "id 1~5 红 / id 6+ 蓝" 写反了**（已在新分支修掉，见 commit "改正颜色过滤的 id 约定注释"）。
- **F2 颜色是两个旋钮**：`detect_color`（感知层报哪些灯条）/ `target_color`（tracker 收哪些 id）。只改一个必然出现"detector 读得到、tracker 读不到"。sentry 默认 `target_color: 1`（打蓝）。
  实测现场：detector 一直发 `ID: 8`，tracker 因为 `target_color=1` 全部 `continue`，输出恒为 `0.00/0.00`。
- **F3 detector 的 `up` 是图像下方那个端点**（`RotatedRect::points()` 的 p0–p3 是宽边；`adjust()` 后 |角度|≤35°，cos>0 恒成立；7000+ 帧录制数据 100% 成立）。
  ⇒ 物体坐标系 +Z 实际指向相机，`pnp.cpp` 里 `min(ang(n, -tvec))` 的**符号是对的**，不要去"修"它。**若有人改了灯条端点顺序/物体点顺序，必须同步翻转这个先验。**
- **F4 IPPE 两解是关于视线的镜像**：在 tracker 世界系里 `(yaw0+yaw1)/2 − 板方位角` 的 p50≈0.0°、std 0.6~2.0°，而两解本身差 20~70°、法线 pitch 差 ~37°。
  ⇒ "两解与 `-tvec` 的夹角"这类判据**结构上无法消歧**（实测两解夹角差 p50 仅 **0.07°**）。消歧只能靠**位置/几何**或时序。
- **F5 半径 `r` 不可观**：单块板只约束"车心在半径 r 的圈上"，位置残差一旦自洽就不再提供 r 信息 → EKF 里的 r 随机游走到 clamp。
  实测：id8 有 **51% 采样贴死 0.40 上限**（而表值应为 0.30/0.27），r 决定虚拟四板的圈半径和自旋瞄准点偏移 → **"位置偏"的直接来源**。
- **F6 换板的物理特征**：同一机器人换板时**车心几乎不动、板跳 2r≈0.6m**；而 yaw 差常只有 20~40°（镜像二义把它压小了）→ **只看 yaw 门限判换板会漏判**，位置门限又拒 → 状态一路 coast 到 LOST（实测匹配率仅 16.9%，p_diff p50 0.65m）。
- **F7 CNN 是 4 类 logits、没有 Softmax**（`cv2.dnn` 读 RM_Armor_CNN_sim.onnx 可验证，最后一层 `head.4/Gemm`，输出 `(1,4)`）。代码只接受 class 0~2，第 4 类被 `is_armor` 里的 `class_id<3` 静默丢弃；`Result::confidence` 注释写"softmax 概率"是错的，且全仓库**只用于画图**。
- **F8 实车 PnP 重投影残差量级** p50≈1px / p95≈3px（角点是整数、±0.5px 量化），可用来判断 IPPE 解是否有效。
- **F9 离线脚本必须用实机同款内参**：detector 与 tracker 都强制**主点=图像中心 (720,540)**，不是标定值 (667.7, 564.2)。用错 K 会让 yaw 统计失真。
- **F10 坐标陷阱（我踩过）**：`/imu/rpy` 是原始 yaw，tracker 内部用的是**开机归零后的 yaw**，两者差一个绕世界 Z 的旋转（实测 46.9°）。
  ⇒ 拿日志里的 `ctr` 与实测板位算距离前，**必须先拟合帧偏置**：扫描 Δyaw 使 `median(| |ctr−板| − r |)` 最小；拟合不出（残差仍 >0.3m）才是真脱节。

---

## 3. 待修清单（按优先级）

> 每条格式：现象 → 根因 → 最小改动 → 验证。

### P0 tracker 匹配门限（实车"飘"的元凶）
- 现象：目标在画面里稳定可见，tracker 却周期性 `TRACKING→TEMP_LOST→LOST`；匹配率 16.9%；车心在日志里来回跳。
- 根因：换板/快速移动时 `p_diff > max_match_distance(0.3)` 被拒，而 `yaw_diff > 57°` 又够不到 → 两条路都不放行 → 按速度 coast。
- 改法（两种，取其一并实测）：① **判断"观测是否更靠近另一块预测板"**（比较到模型其余 3 块板的距离，`nearest_other + 0.05 < min_position_diff && nearest_other < max_match_distance`）；② 或在位置门限失败但**车心到观测板距离 < 2·r_max** 时直接重锚。
- 验证：匹配率（目标 >80%）、失败归因里 `fail_pos` 占比、`TRACKING→TEMP_LOST` 次数/分钟。
- 基线：修复前匹配率 16.9% → 车心距兜底版 86.9%（离线复算）。

### P1 半径 `r`：钉表值 + 补齐 `robot_list_`
- 现象：`r` 贴在 0.40 clamp（51% 采样），命令-几何偏差 yaw p95 5.4°/pitch 4.1°（指令本身很平滑 → 是模型位置偏）。
- 根因：F5（不可观）＋ `robot_list_` 只有 id 0~7，红板 id 8/11 取不到表值。
- 改法：① `robot_list_` 补 `8:(135,125,0.30,0.27)`、`11:(135,125,0.24,0.22)`（红方镜像蓝方同类）；② 每帧把 `state(8)` 吸附到该 id 的两个表值之一（保留 r1/r2 交替语义），放在锚定之前。
- 验证：`r` 应稳定在 0.27~0.30，不再出现 0.40。

### P2 PnP 板 yaw 的镜像二义消歧
- 现象：正对时车心正确，**斜对时车心绕板偏转**，原地自转时车心移动向量很大。
- 根因：F4；而车心 = `armor + r·(cos yaw, sin yaw)`，镜像解把车心甩到视线另一侧。
- 改法：用**位置**在两个候选里挑（`yaw_b = 2·bearing − yaw_a`，`bearing = atan2(y, x)`，取更接近 `atan2(心−板)` 的那个），带 15° 迟滞防抖；接在 yaw 门限判断、EKF 观测 yaw、换板处理三处。
- 验证：同板相邻帧车心跳动 p95（3mspin 0.076→0.019m、yawtest 0.086→0.033m 为离线基线）。

### P3 掉帧 / 小陀螺
- 现象：5m 处 42% 空帧时进不了 SPIN。
- 根因：滑行期把 `v_yaw` 也按 `exp(-dt/0.2)` 衰减；且 `|vyaw|` 低于门限时 `spin_enter_accum_` **硬清零**。
- 改法：滑行期只衰减线速度（保留 v_yaw）；`spin_enter_accum_` 改软回退（`-= dt` 或按比例回退）。
- 验证：`spin` 触发率、vyaw 轨迹。

### P4 detector 侧
- ① CNN 没有置信度门限（F7）：先确认第 4 类语义（背景类？），再决定门限（logits 需要先 softmax 或改用 max-logit 差值）；大小板要用灯条间距/高度比交叉校验，别让 class 直接决定 230/135 模型。
- ② `find_lights` 里 `sample_count = min(10, length)`，`length==1` 时 `i/(sample_count-1)` 是 0/0→NaN，该灯条被恒丢；角点保留 float（现在是 `static_cast<int>` 截断）。
- ③ 5m 掉帧另查：阈值/曝光/CNN（几何门限实测都过：灯条长 15.5px、板宽 43px、面积 ~60px²）。

### P5 tracker node 时间同步
- `nearest_imu` 用 `(a-b).nanoseconds()` 整数相减（`rclcpp::Time` 时钟类型不同会抛异常且没人接）；加 IMU 最大时间差告警（>20ms 降级并打日志）。
- 保留已验证有效的：dt 用真实帧间隔、IMU 按图像时间戳取最近样本（实测对齐误差 −0.1~−1.3ms）。

### P6 `lut_lookup` 越界
- `fi/fj` 没 clamp：距离侧被调用方兜住，但 **z>+3.5m 会被静默钳位**（v=32 时 5m 处 z=4m 误差 −3.8°、z=5m −11.3°）。加 `std::clamp(fi,0,1)`/`clamp(fj,0,1)` 或对 z 限幅。

### P7 `system_delay` 标定（最大未知量）
- 现状 0.1s，ROS 可见延迟仅 ~5.8ms，剩下是串口+MCU+云台机械。
- 建议：云台阶跃响应 + `/imu/rpy` 记录到位时间；或 3~5m 匀速目标扫 `system_delay` 看落点。

---

## 4. 验证口径（**必须照这个来**，否则会得出错误结论）

1. **采集**：同一段时间同时跑 `live_probe.py` + `debug_probe.py`（30~45s），带 §1 的两个环境变量。
2. **指标 A：匹配率与失败归因**——逐帧判断：无同 id 观测 / 位置门限拒 / yaw 门限拒 / 换板被 cooldown 拒。基线 16.9%（修复前）。
3. **指标 B：状态-观测自洽率** = `| |ctr−板| − r | < 0.1` 的占比（**先做 F10 的帧偏置拟合**）。基线：修复前 32%，含换板判据修复 71%。
4. **指标 C：命令-几何偏差**（只用相机系量，无坐标系问题）= 发布 `yaw/pitch` 与 `atan2(dx,dz)`、`atan2(dy−125mm,dz)` 之差。基线 yaw p95 5.4° / pitch p95 4.1°；期望 ≲1°。
   ⚠️ pitch 的 p50 会偏 −0.6~−0.8°，那是弹道下坠补偿（瞄点在目标上方），**不是 bug**。
5. **指标 D**：`r` 稳定 0.27~0.30。
6. **指标 E**：`TRACKING→TEMP_LOST` 次数/分钟。⚠️ 日志是 1Hz 节流，**打印出来的状态转换只是采样值**，别当全量。
7. **离线复算**：`build/live_eval/replay_eval.py`（EKF 移植）+ 录制 CSV 做变体对比。
   ⚠️ 该脚本自带 `if aid>=6: continue`（当初为蓝方数据写的），**红方数据要自己改 loader**；并且要把"tracker 级门限 `matched`"与"EKF 内部马氏拒绝"分开，否则复算结果会和实机不符。
8. **改动前后必须同口径对比**（同一段数据/同一次采集条件），并把数字写进 commit message。

---

## 5. 别重复踩的假线索

- ✗ "PnP 法线先验符号反了" → 见 F3，符号是对的；真正的缺陷是该判据**无法消歧**（F4）。
- ✗ "minAreaRect 顶点顺序跳变导致长短边颠倒、5m 掉帧" → 实测 up–down 恒沿长边，几何门限全过。
- ✗ "同画面出现 id0(230mm) 和 id2(135mm) 一定是 CNN 误判" → 实测投影长宽比 4.63 vs 模型 3.97，更像**测试板尺寸非标准**。
- ✗ HANDOFF 里"yaw 帧间跳变 p95 52.2°→13.7°"**复现不出来**：用它自己的 `yaw_solutions.py` 得 58.9°，直接统计录制流 59.0°。
- ✗ 拿日志 `ctr` 直接与实测板位比距离（F10 坐标系陷阱）。
- ✗ 每帧**硬锚定**车心 → r 失去可观性（F5）；✗ 每帧 `v *= 0.5` 抑制自旋平移 → 帧率相关（130fps 下 5 帧清零提前量）。
- ✗ 用标定主点的 K 做离线分析（F9）。

---

## 6. 分支起点三选一（先决定再动手）

| 方案 | 命令 | 代价 |
|---|---|---|
| A 从最后一次提交开（最干净） | `git switch -c fix/xxx 91fdf3f` | 会丢掉 10-10 之后**全部**未提交工作（含 detector 的 IPPE/法线、ekf R 模型、ArmorInfo nx/ny/nz 等**已验证有效**的部分），需要按 §2/§3 重新实现 |
| B 从当前工作区开（不丢东西） | 先 `git add -A && git commit -m "wip: 本轮实验存档"` 再 `git switch -c fix/xxx` | 分支带上一堆混在一起的改动，需要先按 §3 清理；**推荐**，因为可以逐条 revert |
| C 从当前工作区开但先清 tracker | 同 B，然后按 §3 P0~P3 把 `rm_tracker/src/tracker.cpp` 的车心/换板逻辑重写一遍 | 最费工，但 tracker 最干净 |

现有未提交改动已全量备份：`build/rollback_1016_2323/`（tracker.cpp / node.cpp / tracker.hpp / node.hpp / pnp.cpp / pnp.hpp / ekf.cpp / ekf.hpp / ArmorInfo.msg / AGENTS.md，各 1 份，共 10 个文件）。

**分支现状（先看这个再决定起点）**：

```
当前分支: Tracker_to_C++  @ HEAD = 91fdf3f "一些建议 thoughts.md"  (2026-10-10 17:56)
最近一次"有内容的"提交: 17560b7 "修正了视觉方发给导航方的数据 2.修改了usb相机的参数" (2026-10-14 18:29)
   —— 只在 remotes/origin/nav_interface 上，本分支没有
本分支相对 17560b7: 104 files changed, +10991 / -3832（两条线已经分叉，不要直接 merge）
```

也就是说：**本分支 HEAD 之后（10-10 起）的全部工作都是未提交的工作区改动**，没有可回滚的时间点快照，只能"另开分支 + 按 §3 逐条重做/清理"。

---

## 7. 提交节奏

- 一个补丁一个 commit；commit message 中文短句（如"修改tracker匹配门限"），**并在 message 里写清改了哪条、哪个指标从多少变到多少**。
- 每次提交前 `colcon build --packages-select rm_tracker`（或对应包）必须过；验证没通过就 `git revert` 该 commit，不要累积。
- 验证脚本与采集数据都放 `build/`（gitignore），不要提交进仓库。

---

## 8. 决策点（**已拍板，2026-10-16 晚**）

1. **打红板**：四个兵种配置统一 `target_color: 0`（detector 红板 id = 6/8/11）。
2. **`follow_decision` 全部关闭**：颜色固定由 `target_color` 决定，不跟随 `/nav/decision`（比赛换边时手改配置）。
3. **`r` 钉表值**：每帧把 `state(8)` 吸附到 `robot_list_` 里该 id 的两个表值之一（已提交 `4e68f9e`）。
4. **换板判据用 `nearest_other`**（到其余 3 块预测板的最近距离），不用"车心距兜底"。
5. **可以改 `rm_interfaces`，但每次改动必须先报批**（当前不需要改：`ArmorInfo` 已有 `nx/ny/nz`，而颜色链路按第 2 条不跟裁判系统）。

已落地的提交（分支 `fix/tracker-position-1017`）：`fba1227` 注释纠错 → `4e68f9e` r 钉表值 → `0630354` 验证工具 → `ee0f3b3` 配置统一打红+关 follow_decision → `74e8e91` nearest_imu 整数时钟差+max-age 告警 → `dd6e964` LUT 插值 clamp → `32f0d5d` SPIN 进入累计软回退。

---

## 9. 可复用脚本（`build/live_eval/`，gitignore，开新分支不会丢）

| 脚本 | 用途 |
|---|---|
| `live_probe.py` / `debug_probe.py` | 实车采集（装甲+IMU+云台 / 四角点），**需 UDP 环境变量** |
| `compare_runs2.py` | 帧偏置拟合 + 状态-观测自洽率（跨版本对比，F10 的正确口径） |
| `check_live_frame.py` | 命令 vs 几何期望 + 命令帧间抖动 |
| `verify_mirror_world.py` | 证明"两解关于视线镜像"（F4） |
| `verify_prior_tie.py` | 证明"法线夹角判据是平局"（F4） |
| `diag_live_match.py` / `diag_live_new.py` | 匹配失败归因（指标 A） |
| `replay_eval.py` | EKF 离线回放（注意红方 loader 问题） |
| `verify_lut_range.py` / `verify_onnx_out.py` / `verify_size_model.py` | LUT 越界 / CNN 输出 / 大小板尺寸自洽性 |
