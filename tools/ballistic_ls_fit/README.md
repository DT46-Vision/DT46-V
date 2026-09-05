# 弹道参数离线拟合说明

这套脚本**不参与** ROS / colcon 编译，用本机 Python 跑即可。  
依赖：`numpy`、`scipy`、`pyyaml`（`pip install numpy scipy pyyaml`）

---

## 怎么跑

1. 按下面格式准备 CSV  
2. 编辑 `fit_config.yaml`：改初值，把本次要估的参数 `fit: true`，其余 `false`  
3. 执行：

```powershell
cd D:\project\Python\DT46-V\tools\ballistic_ls_fit
python fit.py --config fit_config.yaml
```

---

## 采数流程

1. 静靶，车体尽量不动。  
2. **手动**把云台调到能稳定连续命中的姿态，保持稳定。  
3. 连续采若干帧（如 30–50），对下面 7 项分别取平均，写成 CSV **一行**。  
4. 换距离 / 高度，重复（近、远都要有）。  
5. 只保存确认打中的时段。

---

## 数据格式（只要这 7 项）

```text
x_cam,y_cam,z_cam,imu_roll,imu_pitch,imu_yaw,v_bullet
```

| 列名 | 含义 | 单位 |
|------|------|------|
| `x_cam` | 目标相机系 X | 米 |
| `y_cam` | 目标相机系 Y | 米 |
| `z_cam` | 目标相机系 Z | 米 |
| `imu_roll` | 陀螺仪 roll | **度** |
| `imu_pitch` | 陀螺仪 pitch | **度** |
| `imu_yaw` | 陀螺仪 yaw | **度** |
| `v_bullet` | 弹速 | 米/秒 |

- 相机系与线上一致（注释：x 右、y 下、z 前）。  
- 默认 CSV 填裸 `/imu/rpy`；`fit_config.yaml` 里 `apply_rotation_rpy: true` 时会按车上的 `rotation_rpy_*` 做叠加（不做开机相对 yaw）。若已是 tracker 内部姿态，把 `apply_rotation_rpy` 设为 `false`。

---

## yaml 开关（`fit_config.yaml`）

每个参数：

```yaml
k_v2:
  value: 0.019   # 初值 / 固定值
  fit: true      # true=本次优化；false=计算时仍使用，但不改
```

可调项：`k_v2`，`cam_to_gun_pos_x/y/z`，`cam_to_gun_rpy_r/p/y`（rpy 单位为**度**）。

- **计算残差**：始终用全部参数 + 全部观测量，解 `(Δyaw, Δpitch)`，期望 `(0,0)`。  
- **优化**：只动 `fit: true` 的项。  
- 注意：当前车上 `solve_ballistic` **未使用** `cam_to_gun_rpy_r`，一般保持 `fit: false`。

写回：`src/rm_tracker/config/tracker_params_*.yaml`。

---

## 文件

| 文件 | 说明 |
|------|------|
| `fit.py` | 统一拟合入口 |
| `fit_config.yaml` | 数据路径、初值、开关、边界 |
| `sample_data.csv` | 7 列示例（假数据，仅试跑） |
| `README.md` | 本说明 |

## 暂不包含

`system_delay`、动目标提前量、bag 一键采集。
