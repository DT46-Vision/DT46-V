# 静止画面的原始 PnP 与观测关联复核

基线为视觉 `d7e68b4`，分支 `codex/relative-aim-restart`；仿真为 `e704b11`，未修改仿真源码或预设。调查对应用户 t≈77.72 s 的截图，原始会话为 `../../../outputs/vscode/aim_hp_review_20261029_185553/cameras/0/`。真值仅作离线评分，不输入在线 PnP 或 tracker。

## 结论与证据

不能把画面中的四块 EKF 名义板直接当成原始 PnP。截图附近 t=77.717 s，原始 PnP 的完整三维法线误差为 0.234°，板中心位置误差约 18 mm；正确的板→心 yaw 为 179.84°，旧车体却保持在 −113.23°。旧关联枚举原 yaw、关于视线的反射和各自的 π 翻转，按预测值选择最接近者，再拿改写后的 yaw 更新；因而可以把正确观测改成错误测量，并长期报告接近零的创新和“匹配成功”。这属于关联前的观测改写故障，不是通过调整位速/角速 EKF 的 Q/R 可以解决的问题。

与此同时，原始 PnP 本身确实仍有静止偏差：此会话启动时约 11.28°。截图后段 PnP 较准不能证明前端已全部修复；旧四板显示失真也不能反过来证明每一帧 PnP 都错。

用完全相同的 4,743 帧和逐项相同的 `observations` 回放，仅修正关联行为，74–88 s 的车心水平 RMSE 从 27.87 cm 降到 7.38 cm；截图帧朝向恢复到 179.84°。没有更改延迟、PnP 尺寸或 Q/R。见 [replay-comparison.json](replay-comparison.json)，本机图像为 [same-input-comparison.png](same-input-comparison.png)。这里的车心 z 是板高状态，不是底盘离地高度，评分只用水平位置。

## 实际修改

1. `BodyEkf::update` 保留 PnP 已选定并经完整变换的 yaw，仅归一化角差；四板枚举、NIS、候选分差和重复观测门控继续工作。去掉会按预测伪造法线的反射/π 翻转。
2. `Tracker` 去掉依赖上述改写的初始镜像车体、累积似然选择及其附加发弹门控。其余当前观测、TRACKING、协方差、弹道与预测一致性门控继续执行。
3. 回归改为验证错误法线被拒绝、正确法线能纠正镜像先验；切板噪声回归继续注入间歇视线镜像、π 错误、漏检、重复及离群观测。镜像注入由每 7 帧中的 3 帧调整为每 31 帧一次，保留偶发错误拒绝和恢复检查，不再要求持续镜像与原始法线等价。旧测试要求“反射观测必须产生与原始观测完全相同的结果”，实际固化了本次故障，不能继续作为正确性标准。
4. 离线 `dt46_armor_geometry_check` 增加 `--all`，审计整段静止靶的观测尺寸；默认仍检查首段 0.6 s。`dt46_pnp_check` 增加可选灯距/灯高，用同角点比较标定候选。两者都不改变在线算法。
5. 算法维护说明同步到 `src/rm_tracker/README.md`，原生验证入口同步到 `standalone/README.md`。本报告是本次测量快照，未改写此前报告。

## 原始 PnP 检查及未完成事项

已有 K/D 与光学变换、已知 15° 装甲安装平面下，对当前会话 3,760 次合格观测做射线/平面相交。检测端点的有效灯距 P05–P95 为 130.18–134.68 mm，灯高为 53.96–57.39 mm；统计见 [geometry-summary.json](geometry-summary.json)。这不是 3,760 个独立视角，也不是实物标定。目标静止，但操作者曾移动相机。

这些检测矩形端点与固定 135×58 mm 的点模型存在不一致；现有观测还混有遮挡和像素轮廓误差。不能仅凭这项审计把所有误差归结为尺寸、渲染或相机标定中的某一项。`inspect_geometry.py` 另投影了灯罩网格外轮廓；该几何外轮廓没有处理遮挡、光照和阈值，不能冒充真实可见灯条的端点。

用前 40 s、朝向余弦大于 0.9 的 746 次观测取中位数，试验尺寸 130.415×56.982 mm：本次手动会话的 PnP 位置 RMSE 从 42.35 降到 26.92 mm，完整法线 RMSE 从 4.55° 降到 3.93°；启动阶段仍有约 5.09°。在独立既有静止闭环样本，法线 RMSE 仍约 8.10°。直接使用灯罩整体外包框中心距/高度也更差。这些结果否定“换一组固定尺寸即可全部修复”，没有将拟合数值写入默认配置。见 [pnp-geometry-comparison.json](pnp-geometry-comparison.json)。

因此，原始 PnP 的静止系统偏差、遮挡后的角点可靠性及跨视角几何一致性尚未解决。单块静止板也不能唯一观测真实半径和车心；修复后的 7.38 cm 不能宣称为车心精度通过。需要稳定的物理特征点对应与跨视角标定证据，不能继续靠调 EKF 噪声、反射法线或只挑重投影更小的结果掩盖它。

## 验证

- 原生 CTest 10/10，见 [ctest.txt](ctest.txt)。新增正确法线纠正镜像先验的回归通过；没有进行 ROS、实机或新的键鼠操作验证。
- 四组既有真实图像日志固定输入回放，车心 RMSE：静止 0.0831→0.0623 m、原地旋转 0.0371→0.0242 m、横移加匀速旋转 0.2094→0.1855 m、正弦平移加旋转 0.2110→0.1944 m。角速度 RMSE 分别为 0.185→0.651、0.724→0.203、0.907→0.355、2.600→0.900 rad/s，短时静止项退步，不隐去。两版关联成功的帧集合可能不同，完整统计见 [four-case-replay.json](four-case-replay.json)；不能解释成每一帧都变好。
- 新九组合闭环各 20 s，使用真实渲染 RGB、检测、PnP、EKF、弹道与 350 HP；共 13,389 帧、175 次实际发弹，九场靶车均扣至 0 HP，无无效发弹许可、无窗口错误，主机和视觉均正常退出。逐场结果见 [live-summary.csv](live-summary.csv)，门控与进程结果见 [live-checks.json](live-checks.json)。这是功能验证，不是上述精度问题通过的证明。`hits` 是累计接触指标，不用于计算独立弹丸命中率。原始目录为 `../aim_model_validation_20261029/live-20261029-191051/`。

已将同一构建的 `dt46_vision.exe` 和 `dt46_aim.dll` 更新到默认 `build/native/bin`；SHA256 对照见 [identities.json](identities.json)。本次自动化场景已结束，未另开人工操作窗口。

复现同输入回放和角点评分（先激活仿真 `.local/runtime-path-check/activate.ps1`，并将 `build/aim-validation-full/standalone` 加入 PATH）：

```powershell
$camera = '../outputs/vscode/aim_hp_review_20261029_185553/cameras/0'
build/aim-validation-full/dt46_tracker_log_check.exe "$camera/vision/frames.jsonl" src/rm_tracker/config/tracker_params_infantry.yaml outputs/pnp_static_diagnosis_20261029/repeat-replay
build/aim-validation-full/dt46_pnp_check.exe "$camera/vision/frames.jsonl" "$camera/truth.jsonl" outputs/pnp_static_diagnosis_20261029/repeat-pnp
build/aim-validation-full/dt46_armor_geometry_check.exe "$camera/vision/frames.jsonl" "$camera/truth.jsonl" outputs/pnp_static_diagnosis_20261029/repeat-geometry.json --all
```

`compare_replay.py` 核对逐帧观测未变并生成对照图；它按本次固定静止目标中心 `[0,-2]` 评分，不用于其他场景。重跑闭环用 `../aim_model_validation_20261029/run-cases.ps1 -Seconds 20`，该脚本启动自动化验证副本，不能作为人工驾驶入口。

偏离原方案：无；本次检查明确分离了原始 PnP 与后续观测关联。未完成：上述原始 PnP 与运动精度问题仍存在，不能声称“全部修好”。
