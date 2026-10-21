# 实车录 bag 检查表（小陀螺 / 换板调参）

用于验证 `spin-wait` + 换板滞回。参数默认 **sentry**（`target_color: 0` 打红；打蓝改 `1`）。

## 1. 录之前

- [ ] 机载已 `source install/setup.bash`，sentry launch 能正常出图、出 IMU
- [ ] 确认敌人颜色与 `target_color` 一致
- [ ] 磁盘空间：每条约 1–3 分钟图像 bag 预留数 GB
- [ ] 记下：机器人型号、弹速设定、相机、大概距离、室内/外

## 2. 必录话题

```text
/image_raw
/camera_info
/imu/rpy
```

联调追踪时若现场要直接看效果，可再开 `display:=true`，但 **素材 bag 不必录调试图**（体积大）。事后用本检查表素材 + detector/tracker 回放即可。

录制示例：

```bash
ros2 bag record -o bags/raw/<name> \
  /image_raw /camera_info /imu/rpy
```

## 3. 场景清单（按优先级）

命名：`YYYYMMDD_<scene>_<note>`，例如 `20261020_spin_const_2m`。

| 优先级 | scene 名 | 目标怎么动 | 时长 | 通过标准（回放时看） |
|--------|----------|------------|------|----------------------|
| P0 | `spin_const` | **原地匀速**小陀螺，尽量转速稳定 | ≥ 12 s 有效转 | 进 SPIN；瞄点钉在车身正前附近；FIRE 呈间歇窗口而非常亮 |
| P0 | `spin_var` | **原地变速**小陀螺（忽快忽慢 / 停一下再转） | ≥ 15 s | SPIN 保持或合理进出；换板时 yaw 不剧烈来回撕 |
| P1 | `spin_trans` | 小陀螺 + **慢速平移**（横移或前后，位移明显到米级） | ≥ 15 s | 车心跟着走；正对方向不漂飞；仍能出现 FIRE 窗口 |
| P2 | `spin_irregular` | 边转边变向 / 急加减速 / 蛇形 | ≥ 12 s | 不长期丢跟；少误出 SPIN；TEMP_LOST 可恢复 |
| P2 | `trans_only` | **不转**，只平移（对照） | ≥ 8 s | **不进** SPIN；普通 lead 瞄准正常 |

距离建议：**2–4 m**；装甲大部分时间在画面内，能看清换板。

## 4. 每条 bag 现场勾选

- [ ] 开录后再让目标动（或片头留 1–2 s 静止）
- [ ] 有效运动段达到上表时长（不含对准、遮挡）
- [ ] 无明显长时间丢识别（灯条被挡超过 2–3 s 则重录）
- [ ] IMU 有数：`ros2 topic hz /imu/rpy` 正常
- [ ] 图像有数：`ros2 topic hz /image_raw` 正常
- [ ] 停录后 `metadata.yaml` 存在，db3 非空

## 5. 回放验收（录完回实验室）

参数：`detector_params_sentry.yaml` + `tracker_params_sentry.yaml`，`display:=true`。  
CPU 检测请 `--rate 0.15~0.25`；有 CUDA 可接近 1×。

看：

1. `/tracker/ballistic_img`：`SPIN` / `FIRE ENABLE|HOLD FIRE`
2. `/tracker/tracking_state_img`：绿十字是否钉正前、换板是否肉眼抖动
3. `/tracker/gimbal_control`：`can_fire` 占空比、yaw 步进是否炸

可把回放产物放到 `bags/test_<branch>/`（两路 mp4 + 含 gimbal/armors 的 data bag），便于对比调参。

## 6. 和现有素材的关系

| 已有 | 覆盖 |
|------|------|
| `bags/spin_const` | ≈ P0 匀速原地（旧） |
| `bags/spin_var` | ≈ P0 变速原地（旧） |
| `bags/raw/20261020_spin_const_p0` | P0 匀速，~16.8 s，image/imu/camera_info |
| `bags/raw/20261020_spin_var_p0` | P0 变速，~19.3 s，image/imu/camera_info |
| `bags/raw/20261020_spin_trans_p1` | P1 陀螺+平移，~17.6 s，image/imu/camera_info |
| （缺） | P2 不规则 / 纯平移对照 |

**调换板滞回**：P0 两条够用。  
**说能上场**：至少补一条合格的 `spin_trans`。
