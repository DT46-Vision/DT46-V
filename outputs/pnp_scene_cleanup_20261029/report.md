# 原始 PnP 与场景问题复核

视觉基线 `be1992b`，工作分支 `codex/relative-aim-restart`。场景菜单及正式预设的改动归 [仿真报告](../../../outputs/scene_preset_cleanup_20261029/report.md)。本报告不把能够发弹等同于精度全部通过。

## 已修复与显示

1. 原 IPPE 双解选择用上一帧法线、深度及近似尺寸深度打分，不比较当前角点重投影误差，能够持续选择更差的分支。现在检查四个角点的正深度及有限数值，按当前四角像素残差选解，移除跨帧法线锁定。仍使用已有 IPPE 的有限候选，不新增在线迭代求解器。
2. `use_geometric_center` 原来通过浅拷贝修改缓存 K，后续关闭该选项仍继续使用被改写的主点。现在对本次 K 做独立副本，回归检查覆盖切换后恢复标定。
3. 原生逐帧观测新增 `pnp.corners_camera_m` / `pnp.reprojection_error_px`。跟踪图独立显示黄色检测端点、品红原始 PnP 重投影和 120 mm 法线箭头、青/绿/橙 EKF 模型。文字移到画面边缘并缩短模型标签，避免互相遮挡。原生 C ABI 未修改；ROS 发布消息未扩展。
4. 新增离线 `dt46_pnp_check`，逐项记录 IPPE 两个候选的像素误差、位置误差及完整法线误差；真值只用于离线评分，没有进入在线求解或 EKF。

检测器历史 `up` 实际是图像下端，两个灯条的绕序一致。它与朝外法线约定相配，不把上下标签反命名直接当成 PnP 反序故障；本次只更正相关注释，没有翻转分类输入。

## 同角点比较

假设：旧时序打分会压过当前透视信息。使用同一批 `live-20261029-180906` 已记录角点及 K/D；两版均用 135 × 58 mm 灯条尺寸，不修改 EKF 参数。最近离线真值间隔不超过 20 ms；取最近板中心距离小于 0.2 m 的观测，按现有 AM02 模型 15° 外倾构造真实法线。法线误差为完整三维夹角，不枚举镜像取最好结果。

| 固定输入 | 观测数 | 旧版选择较大像素误差的次数 | 完整法线 RMSE：旧 → 新 | 重投影 RMS：旧 → 新 |
|---|---:|---:|---:|---:|
| 静止不转 | 73 | 15 | 0.201918 → 0.201259 rad | 0.351575 → 0.272057 px |
| 原地匀速旋转 | 430 | 134 | 0.490372 → 0.082859 rad | 1.604415 → 1.058127 px |

数值见 [pnp-comparison.json](pnp-comparison.json)，原始逐候选结果留本机 `before-*` / `after-*`。旋转样本支持双解选择故障及本次修复；静止样本否定“改选解即可消除全部误差”的假设，仍有约 11.5° 法线 RMSE 和 18.3 mm 位置 RMSE。

之前静止光学样本推导的 130.282 × 57.129 mm 在新选择器下再次重放，静止观测位置 RMSE 可降至约 4.75 mm，但匀速横移加旋转的角速度 RMSE 约 4.00 rad/s；未将这组单视角拟合尺寸写入默认参数。结果在 `replay-*`，不是实物标定，也不能证明尺寸跨距离、角度和光照通用。

## 实际运行

- 原生 CTest 10/10，通过新增 12 种精确投影姿态的法线和重投影检查，以及几何主点切换检查；见 [ctest.txt](ctest.txt)。ROS / 实机未验证。
- 九种平移/旋转组合各 20 秒，真实渲染 RGB → ONNX 检测 → PnP → EKF → 弹道 → HP，合计 11,416 帧、164 次实际发弹；九场均发弹扣血、正常退出、无无效发弹许可、无窗口错误。运行是自动化验证副本；用户入口仍为人工控制。见 [live-summary.csv](live-summary.csv)、[live-checks.json](live-checks.json)。`hits` 是原仿真累计接触指标，本报告不把它当作独立弹丸命中数。
- 四个本次闭环的离线精度见 [accuracy.csv](accuracy.csv)：静止、原地旋转的车心 RMSE 约 0.083 / 0.037 m；横移加旋转约 0.209 / 0.211 m。闭环采样和轨迹随在线控制变化，这些数值不能作为同角点 A/B 精度比较。
- 静止、人工控制、未开火时保存了原始 PnP 叠加图。本机原图：`../../../outputs/vscode/pnp_manual_check_20261029_184603/cameras/0/vision/frame_g0_r2_n99_t1.215000/tracking_state_img.png`；`static-pnp-overlay.png` 仅裁剪，没有改算法结果。此帧重投影 0.34 px、深度 1.03 m；贴合检测点不代表法线或物理尺寸已经准确。

验证使用 `build/aim-validation-full`；已把 `dt46_vision.exe` 和配套 `dt46_aim.dll` 部署到默认 `build/native/bin`，身份记录见 [identities.json](identities.json)。原始运行目录为 `outputs/aim_model_validation_20261029/live-20261029-184345` 与 `live-20261029-184623`，日志和图像留本机。

复现固定角点评分：

```powershell
cmake --build build/aim-validation-full --target dt46_pnp_check
build/aim-validation-full/dt46_pnp_check.exe <frames.jsonl> <truth.jsonl> <新输出目录>
```

重跑九场闭环：加载仿真 `.local/runtime-path-check/activate.ps1`，把 `build/aim-validation-full/standalone` 加入 PATH，执行 `outputs/aim_model_validation_20261029/run-cases.ps1`；该脚本已有全部默认组合。旧版数值必须使用基线 PnP 源码，不能用新版工具重跑后冒充旧版。

偏离原方案：无。未完成：静止法线的系统偏差、远距离/运动观测偏差及其车心关联影响仍未消除，PnP 与 EKF 精度未全部通过。需要进一步取得跨距离、跨角度的有效灯条端点标定，不能靠本次重投影变小或多次命中认定修复完成。
