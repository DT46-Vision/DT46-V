# 弹道最小二乘参数拟合

分支语境：`Tracker_to_C++`。不改 `git remote`。

## 目标

用静态命中数据，最小二乘估计弹道/外参中手调不稳的量，写回 ROS 参数后重建 `build_ballistic_lut()`。

对齐现有 C++ 物理（`src/rm_tracker/src/tracker.cpp`）：

- 平方阻力 + 重力：\(a_x = -k_{v2} v v_x\)，\(a_z = -g - k_{v2} v v_z\)
- 欧拉积分 `dt=5ms`，打靶调仰角
- 运行时用 LUT；拟合时用**同一积分器**算残差，不用 LUT 插值当真值

## 拟合参数

### 第一轮（推荐只估这两个）

| 符号 | 对应代码/参数 | 含义 |
|------|----------------|------|
| \(k_{v2}\) | `Tracker::k_v2` | 平方阻力系数 |
| \(b_p\) | `cam_to_gun_rpy` pitch（度→弧度） | 俯仰常值偏置 |

### 第二轮（有多距离/多高度后再加）

| 符号 | 对应 | 含义 |
|------|------|------|
| \(p_z\) 或竖直 lever | `cam_to_gun_pos` 相应轴 | 相机–枪口高度差（改枪口系 \(z\)） |
| \(s_v\) | \(v = s_v \cdot v_{\text{C板}}\) | 弹速比例（与 \(k_{v2}\) 强耦合，慎用） |
| \(b_y\) | `cam_to_gun_rpy` yaw | 偏航零位（可单独一维拟合） |

### 固定 / 静态不估

- `g = 9.81`
- `system_delay`（只影响运动提前，静靶不可辨）
- EKF / 选板 / 开火门限

## 残差（与实车一致的定义）

**采集的是原始量，不是手算好的枪口系距离。**

每个样本 \(i\)：静靶下**手动瞄到能稳定命中**，多帧平均：

1. 视觉装甲板（相机系）
2. 陀螺仪 / C 板姿态
3. C 板弹速

用待估参数 \(\mathbf{p}\) 跑与车上相同的链：

视觉 + IMU → 枪口系 → 打靶积分 → 相机系瞄准角 → \((\Delta\text{yaw},\,\Delta\text{pitch})\)

因为已经瞄对并能命中，**正确 \(\mathbf{p}\) 下应有**

\[
(\Delta\text{yaw},\,\Delta\text{pitch}) \approx (0,\,0)
\]

残差（两路可拆开）：

\[
r_i^{(p)} = \Delta\text{pitch}_i(\mathbf{p}),\quad
r_i^{(y)} = \Delta\text{yaw}_i(\mathbf{p})
\]

\[
\min \sum_i \big(w_p [r_i^{(p)}]^2 + w_y [r_i^{(y)}]^2\big)
\]

实务上仍建议：**先只优化俯仰相关**（`k_v2`、`rpy_p`），**再优化偏航**（`rpy_y`）。

说明：枪口系 \(d_h,z\) 只出现在脚本**内部**变换结果里，不是让操作员手填的「观测」。  
若离线 CSV 暂存 \(d_h,z\)，那只是把变换提前做完的调试简化，正式流程应存原始视觉 + IMU + 弹速。

等价视角：命中姿态下，自瞄输出的修正角就是误差；标定就是把这些修正拧到 0。

## 数据采集

静靶、车不动。

单点流程：

1. **手动**调云台到能轻松连续命中，保持稳定
2. 连续采 N 帧（建议 30–50）：视觉原始、IMU 原始、弹速
3. 平均 → 1 个数据点；只保留确认命中时段
4. 换距离/高度，重复（近+远）

建议原始字段（一点一行）：

```text
armor_x_cam, armor_y_cam, armor_z_cam, armor_yaw_cam,
imu_roll, imu_pitch, imu_yaw,
v_bullet, n_frames, hit_ok
```

（可选对照：当时的 `gimbal_yaw_cmd`, `gimbal_pitch_cmd`，拟合目标仍是期望为 0。）

## 脚本位置

`tools/ballistic_ls_fit/`（仓库根下，不进 colcon）

- `fit_pitch.py`：第一路 `k_v2` + 俯仰偏置
- `fit_yaw.py`：第二路 偏航偏置
- `sample_data.csv`：字段示例

## 实现步骤

1. 按 CSV 字段采静靶命中数据，替换/扩展 `sample_data.csv`
2. `python fit_pitch.py --csv your.csv` → 再 `python fit_yaw.py --csv your.csv`
3. 写回 `src/rm_tracker/config/tracker_params_*.yaml`；实车确认 LUT 重建
4. （可选）rosbag/话题一键录静靶段

不在本计划内：改 remote、动目标 delay 标定、重写在线求解器结构。

## 与在线代码的衔接

- 拟合结果只更新参数，不改 `solve_ballistic` 控制流
- `bullet_speed` / `k_v2` 变更后必须 `build_ballistic_lut()`（节点里已有弹速更新时重建逻辑）
- 近距 `<1.5m` 在线走直线瞄准；拟合样本仍应用全模型，或近距点权重主要用于 \(b_p\)

## 约定

- 不修改 git remote
- 改动保持最小；拟合工具可先放独立脚本/目录，确认后再接入仓库结构
