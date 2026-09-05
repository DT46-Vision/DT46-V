# 弹道最小二乘参数拟合

分支：`feat/ballistic-ls-fit`（基于 `Tracker_to_C++`）。不改 `git remote`。

以 **`tools/ballistic_ls_fit/README.md`** 为准。

目录：`python/` 脚本，`config/` 拟合参数，`data/` CSV；根目录放 README、`record.sh`、`fit.sh`。

---

## 目标

1. 静靶命中时采原始量，离线非线性最小二乘估外参 / 空阻。  
2. 写回 `tracker_params_*.yaml`，实车重建 `build_ballistic_lut()`。  
3. **采数**：按一次按钮 / 调一次服务 → CSV 追加一行 → 再丢给 `fit.py`。

---

## 已完成

### 离线拟合

| 项 | 说明 |
|----|------|
| 入口 | `python/fit.py`，配置 `config/fit_config.yaml` |
| 采数 | `python/record_csv_node.py` / `./record.sh` |
| 拟合启动 | `./fit.sh` |

---

## 残差与采数约定

- CSV 仅 7 列：`x_cam,y_cam,z_cam,imu_roll,imu_pitch,imu_yaw,v_bullet`（m / 度 / m/s）  
- 手动命中后再记；不手测「应转多少度」  
- 计算用全参数；yaml 开关决定估谁  

---

## 后续可选

- 正式 rqt 插件（当前用 tk 小窗 / service caller 即可）  
- 按键时对最近 N 帧取平均再写  
- `system_delay` / 动目标；改 remote；改线上求解器结构  

---

## 相关路径

| 路径 | 用途 |
|------|------|
| `tools/ballistic_ls_fit/README.md` | 用法 |
| `python/` `config/` `data/` | 脚本 / 拟合配置 / CSV |
| `record.sh` `fit.sh` | 启动 |
| `.cursor/doc/ballistic-and-lead.md` | 阻力 vs 提前量 |
