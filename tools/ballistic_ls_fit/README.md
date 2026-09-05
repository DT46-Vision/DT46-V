# 弹道参数离线拟合 / 采数

不进 colcon。依赖拟合：`numpy` `scipy` `pyyaml`；采数还需已 source 的工作空间（`rm_interfaces`）。

## 目录

```text
ballistic_ls_fit/
  README.md          # 本说明
  record.sh          # 采数启动
  fit.sh             # 拟合启动
  python/            # Python 脚本
  config/            # 拟合配置（开关、初值）
  data/              # CSV（示例 + 实采）
```

参数放在 **`config/`**（和 CSV 分开：一个是拟合设定，一个是观测数据）。

---

## 三种用法

### 1. 平时自瞄

照常 launch（如 `infantry.launch.py` / 你的 `start.sh`），**不用**本目录。

### 2. 采数据（写 CSV）

先开着自瞄，再：

```bash
cd <仓库>/tools/ballistic_ls_fit
chmod +x record.sh    # 首次
./record.sh
```

启动后终端会列出 `data/` 里已有的 `.csv`，输入序号选择；选 `0` 可新建。  
然后弹出小窗，点「记录一点」追加到所选文件。

跳过交互、直接指定文件：

```bash
CSV_PATH=$PWD/data/run1.csv ./record.sh
```

### 3. 算参数（离线）

```bash
cd <仓库>/tools/ballistic_ls_fit
# 可先改 config/fit_config.yaml 里的 fit 开关
./fit.sh
```

启动后同样从 `data/` 列表里选一个 CSV。也可：

```bash
./fit.sh --csv data/sample_data.csv
```

Windows：

```powershell
cd tools\ballistic_ls_fit
python python\fit.py --config config\fit_config.yaml
python python\record_csv_node.py --ros-args -p enable_gui:=true
```

---

## CSV 格式（仅 7 列）

```text
x_cam,y_cam,z_cam,imu_roll,imu_pitch,imu_yaw,v_bullet
```

单位：位置 **m**，姿态 **度**，弹速 **m/s**。

---

## 拟合配置（`config/fit_config.yaml`）

- **不写 CSV 路径**；启动时从 `data/` 选择  
- 每个参数 `value` + `fit: true/false`  
- 算残差用全部参数；只优化 `fit: true` 的项；期望 `(Δyaw, Δpitch)≈(0,0)`

---

## 文件说明

| 路径 | 说明 |
|------|------|
| `python/fit.py` | 离线最小二乘 |
| `python/record_csv_node.py` | ROS 采数节点 |
| `python/csv_io.py` | CSV 追加 |
| `config/fit_config.yaml` | 拟合开关与初值 |
| `data/sample_data.csv` | 示例 |
| `data/calib.csv` | 实采（运行后生成） |
