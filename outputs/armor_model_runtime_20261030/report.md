# 原生模型识别与定位接线验证

本轮从视觉 `de58a15`、仿真 `2478bf3` 开始，在双方 `codex/armor-pose-training` 上接通模型在线路径。优化 PnP 对照保留于本地附注标签 `2026-10-30-pnp-baseline` → `3c961fb278aab86b4679e95bf90c031fc461a200`。未合并、未推送。

## 实际变化

- `standalone/armor_model.*`：亮度/色差/形状只提出双灯条候选，保留原始透视 RGB 裁图；由同一个模型判定装甲有效性、阵营、编号、XYZ 和旋转。候选在模型评分后去重，不再经过旧数字分类器。已用真实裁图、整帧及实时仿真验证。
- 裁图和内参：128×128 letterbox、RGB(20,20,20) padding、半像素中心变换与数据集一致，D 原样传入。深度、距离、偏角从同一份位姿计算，未新增网络头、损失或训练。几何单测和 CPU 数值对照通过。
- `standalone/aim.cpp`：模型相机系米制位置/法线进入已有坐标变换及车辆 EKF；原 C 结构布局保留，JSON 使用 `presence_score` 与 `model` 明细。基地/前哨站输出分类定位，但不送入四板车体 EKF。非法位姿被拒绝并计数。重复帧实验核对初始化位置、单位和 TRACKING 转移；静止仿真验证通过。
- 构建与配置：在线目标不编译旧 detector、NumberClassifier、PnP；相关旧检查退出默认构建，源码和历史标签保留。原生读取 `standalone/armor_model.json`，ROS 源码及配置保持原用途。旧 PnP/贴纸处理参数会明确拒绝，调参事务失败保留原上下文。
- 导出：新增 `--fixed-batch`，同一 checkpoint 单独导出 batch=1 部署文件。OpenCV 4.12 对原动态 batch 文件的 Concat 导入报形状错误；固定 batch 后可加载并通过真实数据数值对照。未改权重或学习目标。
- 调试显示：取消 PnP 重投影、像素残差、法线箭头以及旧数字二值图；改用模型类别/阵营/深度和 `model_input` RGB 预览。原 EKF/弹道图保留。新窗口打开、保存、关闭及截图检查通过。
- 仿真运行证据：`session_app.cpp` 归档实际原生配置、模型元数据、tracker 配置，并记录 ONNX 与运行库哈希，替代不再生效的旧分类 YAML。实时运行中逐项重算哈希通过。
- 文档：同步 standalone 部署/坐标契约、训练导出说明、旧 ROS detector 的适用范围，以及仿真入口；本报告维护当次验证结果。

## 方法与证据

假设是“旧分类/PnP 可以退出，CPU 模型输出经正确裁图/标定和坐标变换进入现有 tracker”，不是“首版权重已达到瞄准精度”。先验证固定裁图数值一致性，再跑完整验证集及固定参数的独立测试集；最后运行实际仿真与调试窗口，分别记录成功接线和误差。真值只用于离线评分，不进入原生请求。

模型仍为 epoch 18 的 `best.pt`；部署 ONNX SHA256：`bf450957f18d6ba483db47c45135b6851cfaaac1e8b315232b6384f787fb57ed`。完整运行配置、数量和误差见 [validation/summary.json](validation/summary.json)、[test/summary.json](test/summary.json)，程序哈希与检查状态见 [checks.json](checks.json)。检查硬件为 Ryzen 9 8945HX；原生 OpenCV 4.12、CPU 单线程。网络前向与整帧时延不能混用。

| 独立测试项目 | 结果 |
|---|---|
| 同一裁图 C++ / ONNX Runtime 输出对照 | 1,122 / 1,122 通过，rtol/atol=1e-4；最大 XYZ 分量差 0.00001335 m |
| 裁图装甲有效性 | TP 883、FP 7、FN 3、TN 229 |
| 裁图贴纸 / 阵营 | 764/886（86.23%）；875/884（98.98%），只计对应监督有效的标签 |
| 全部受监督正裁图位姿 | 深度绝对误差 P50 0.429 m、P95 1.581 m；XYZ RMSE 0.772 m；旋转均误差 31.11° |
| 1,000 张整帧 | 886 个选中目标，754 个输出，匹配 604、漏匹配 282、未匹配输出 150 |
| 整帧已匹配的贴纸 / 阵营 | 542/604、603/604；同时正确 541/604 |
| 纯负整帧 | 236 张中 22 张有输出 |
| 整帧已匹配目标位姿 | 深度绝对误差 P50 0.473 m、P95 1.676 m；XYZ RMSE 0.824 m |
| 完整原生处理时延（不含读图、重建、相机、UI） | P50 6.06 ms、P95 93.38 ms；每帧候选均值 4.29、P95 28.1、最多 64 |

整帧匹配是预测裁图与选中真值裁图之间 IoU≥0.3 的一对一匹配，**不是标准检测 AP**。未匹配输出包括错误候选和裁图不对应，不全部称为已证实的误识别。整帧位姿只统计匹配成功部分，不能与“全部正裁图”混作同一分母。候选较多时尾部耗时明显，当前不能宣称稳定几毫秒一拍。验证集/测试集各两条无有效阵营监督的正样本已从分类分母剔除；仅重算统计，不重选权重或参数。

## 实时仿真与限制

[最终 18 秒静止场景检查](../../../outputs/armor_model_live_final_20261030/model_checks.json)：897 帧均输出蓝方 `infantry_3`，895 帧为 TRACKING，2 帧为 DETECTING，视觉无错误退出。打开/保存/关闭窗口的上下文地址保持一致；保存的 `model_input.png`、`tracking_state_img.png`、`window.png` 位于该运行目录的 `cameras/0/vision/frame_g0_r2_n26_t0.441000/`，已检查图中无旧 PnP 绘制。

同帧真值核对见 [live_truth/truth-summary.json](live_truth/truth-summary.json)：原始装甲位置误差约 0.213 m，追踪车心误差约 0.240 m。这是固定相机、固定姿态的一组静止结果，不是动态跟踪精度；EKF 未消除模型偏差。该场景视觉 `processing_ms` 的 P50/P95 为 14.47/16.47 ms，包含启用调试图像的实际处理，不等同于离线关闭显示的测试集计时。该运行 `can_fire` 为 0，末帧原因为 `alignment_or_facing` 和 `arrival_probability`；本轮没有为了允许开火而放宽门控。

已通过 `armor_model_geometry`、`native_pipeline`、`tracker_body_integration`、`native_transport` 和仿真 `auto_aim_control`。完整 Windows 操作回归 `workflow_37512` 在画质面板保存后等待 `rendering.json` 超时，未到后续右键跟踪/许可发弹步骤；其已记录 1,046 帧中有 870 帧 TRACKING，但**不能记为全流程通过**。本轮单独执行的新模型窗口生命周期检查通过，不抵消前述失败。

偏离原目标：无。固定 batch 导出是现有 CPU 后端的兼容实现；训练目标不变。未完成的质量项：整帧漏检/假候选、模型测距和姿态误差、复杂画面的推理尾延迟、完整窗口回归及动态/发弹精度验收。未实现独立多车 track 池或结构目标跟踪。ROS、实车按用户要求留到后续，未验证。

## 复现

在视觉根目录，先按 [训练导出说明](../../tools/armor_pose_training/README.md#评估与-cpu-导出)生成部署文件，依赖路径按本机实际环境提供。二进制、图片与逐帧大日志留本机；源码、脚本和统计入库。

```powershell
cmake --build build/native --target dt46_vision dt46_model_eval dt46_armor_model_check dt46_pipeline_check dt46_tracker_model_check dt46_transport_check dt46_aim_truth_check --parallel 4
ctest --test-dir build/native -R 'native_pipeline|armor_model_geometry|tracker_body_integration|native_transport' --output-on-failure
.cache/armor-training-env/Scripts/python.exe outputs/armor_model_runtime_20261030/verify_runtime.py --dataset ../outputs/armor_batch_20261030/dataset_10000 --split test --opencv-bin ../.deps/environment-path-check/install/runtime-path-check/opencv/x64/mingw/bin --output outputs/armor_model_runtime_repeat
```

实时检查在仿真根目录运行，输出必须是新目录：

```powershell
. ./.local/runtime-path-check/activate.ps1
DT46_V/.cache/armor-training-env/Scripts/python.exe DT46_V/outputs/armor_model_runtime_20261030/verify_live.py --root . --output outputs/armor_model_live_repeat
DT46_V/build/native/bin/dt46_aim_truth_check.exe outputs/armor_model_live_repeat/cameras/0/vision/frames.jsonl outputs/armor_model_live_repeat/cameras/0/truth.jsonl outputs/armor_model_live_repeat/truth_check
```

`probe_opencv.cpp` 保留原动态 batch 导入失败的最小复现；它是离线证据，不参与在线构建。
