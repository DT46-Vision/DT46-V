# tools/live_eval —— 实车验证工具（已纳入版本管理，别再放 build/ 下）

> 背景：`build/live_eval/` 里的脚本和录制数据被清过一次（含 live_probe/debug_probe/replay_eval 与全部 CSV）。
> 现在工具放在 `tools/live_eval/`（受版本管理），**数据仍放 `build/live_eval/`**（不进仓库）。
> 方法论、已核实结论、待修清单见 `build/live_eval/PROMPT_NEW_BRANCH.md`（该文件也在版本管理外）。

## 0. 跑任何 python 节点前的两个环境变量（**必需**）

```bash
export ROS_LOG_DIR=/home/glennade/DT46-V/build/roslog   # rclpy 要写日志；home 只读会让 ros2 命令直接失败
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4                 # 容器/沙箱里共享内存被挡，不设这句订阅收不到任何数据
source /opt/ros/humble/setup.bash && source install/setup.bash
```

## 1. 采集（一次同时跑两个，数据必须同段）

```bash
cd build/live_eval
python3 ../../tools/live_eval/live_probe.py  40 run1 &     # -> run1armors.csv / run1imu.csv / run1gimbal.csv
python3 ../../tools/live_eval/debug_probe.py 40 run1       # -> run1_debug.csv（四角点）
```

## 2. 判据（照这个顺序看，别自创口径）

| # | 指标 | 脚本 | 基线（未修） | 目标 |
|---|---|---|---|---|
| A | 匹配率与失败归因 | 需自己按 §3 的记录逐帧统计 | 16.9%（fail_pos 82%） | >80% |
| B | 状态-观测自洽率 | `compare_runs.py run1` | 32%（含换板修复后 71%） | 越高越好，p50<0.05m |
| C | 命令-几何偏差 | `check_live_aim.py run1` | yaw p95 5.4° / pitch p95 4.1° | ≲1° |
| D | 半径 r | `compare_runs.py` 的 r 分布 | 51% 贴 0.40 clamp | 稳定在表值 0.30/0.27 |
| E | 丢目标频率 | tracker 日志里 `TRACKING -> TEMP_LOST` 次数/分钟 | 455s 内 12 次 | 明显下降 |

**三个容易踩的坑**：
1. `/imu/rpy` 是原始 yaw，tracker 用**开机归零后**的 yaw，两者差 40~50° → 拿日志 `ctr` 与实测板位比之前必须拟合帧偏置（`compare_runs.py` 已内置）。直接比会得到"|ctr-板|≈2m"的假结论。
2. tracker 日志是 1Hz 节流，**打印出来的状态转换只是采样值**，不是全量，别用来算比率。
3. 离线复算 PnP 时内参要用**实机同款**：主点=图像中心(720,540)，不是标定值(667.7,564.2)。

## 3. 四角点离线复算（判断 PnP 分支）

`run1_debug.csv` 每行是一块板的四个角点(l1u/l1d/l2u/l2d)，配合 `run1armors.csv` 的同 stamp 行可复算 IPPE 两解：

```python
import cv2, numpy as np
K=np.array([[1647.43487,0,720.0],[0,1646.15979,540.0],[0,0,1.0]])      # 主点用图像中心!
D=np.array([-0.38290473831374955,0.16406490055158807,-0.001390390715926989,0.00020454992210411328,0.0])
op=np.array([[-67.5,-29.,0],[-67.5,29,0],[67.5,-29,0],[67.5,29,0]])     # 小板(135mm)；id 0/6 用 ±115
ip=np.array([[l1u_x,l1u_y],[l1d_x,l1d_y],[l2u_x,l2u_y],[l2d_x,l2d_y]], float)
ret, rvecs, tvecs, reperr = cv2.solvePnPGeneric(op, ip, K, D, flags=cv2.SOLVEPNP_IPPE)
```

已核实：两解是**关于视线的镜像**（世界系里 `(yaw0+yaw1)/2 − 板方位角` p50≈0.0°、std 0.6~2.0°），
两解本身差 20~70°，而两解与 `-tvec` 的夹角几乎相同（p50 0.07°）⇒ **"法线夹角"类判据无法消歧**。

## 4. 提交纪律

- 一个补丁一个 commit（中文短句 + 指标变化写进 message），commit 前 `colcon build --packages-select rm_tracker` 必须过。
- 验证不过就 `git revert` 该 commit，不要累积。
- 采集数据写 `build/live_eval/`（gitignore），工具改动写 `tools/live_eval/`（入版本管理）。
