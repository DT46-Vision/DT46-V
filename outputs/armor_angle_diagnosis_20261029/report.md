# 当前斜视角无法匹配装甲板的原因

视觉 `b9309b4`。原始画面与运行参数留在仿真会话 `../../../outputs/vscode/aim_hp_play_20261029_192040/cameras/0/vision/frame_g4_r8_n69368_t200.309000/`。该帧实际日志为 `light_count=4`、`observations=[]`、`state=LOST`；问题发生在输出装甲板观测之前，尚未进入 PnP 和车体关联。

使用保存的原始 RGB、同一 ONNX、实际 detector 参数离线调用当前检测器，复现 `lights=4 armors=0`，见 [result.txt](result.txt)。`inspect_detector.cpp` 是仅针对该组保存参数的审计工具，没有修改正在运行的程序或参数。

按 x 排序，候选灯条中心分别为 `(626,704)`、`(998,729)`、`(1062,712)`、`(1110,691)`。其中 x≈1062 的长轮廓对应右侧数字“3”的亮笔画，被灰度轮廓、形状和中心线颜色检查误当成灯条。右侧装甲板真正的两条灯是候选 1、3。

| 配对 | 高度比 | 间距/平均灯高 | 当前行为 |
|---|---:|---:|---|
| 0、1 | 1.035 | 4.392 | 大于上限 3，拒绝 |
| 1、2（数字误检） | 1.707 | 0.567 | 小于下限 1，拒绝 |
| 1、3（真实右板） | 1.287 | 1.541 | 几何通过，分类 class_id=1、原始 logit=5.922，当前贪心配对未尝试 |
| 2、3 | 2.196 | 0.488 | 高度比超过上限 1.8，拒绝 |

真实 1、3 的灯条倾角差为 0.821°、中心连线角为 18.741°，均低于 35°。直接调用 `is_close` 可以得到正确数字类别，说明本次右板的拒绝原因不是数字模型无法识别该角度。

实现位置为 `src/rm_detector/src/detector.cpp` 的 `ArmorDetector::is_armor`：先找右侧第一个同色且斜率通过的灯，随后只调用一次 `is_close`；几何或分类失败便进入下一个左灯，不继续寻找当前左灯的后续右灯。数字假灯因此挡住了真实配对。这是误检与贪心候选选择共同触发的确定性反例，不应通过放宽 PnP 或 EKF 门限解决。

本次仅保存复现与结论，没有改变运行中的视角、暂停状态或参数。现有检测器的缺陷尚未在本记录中修复；后续修复应在几何或分类失败后继续检查候选，并保留跨板误配的拒绝约束。

复现：在视觉根目录激活 `../.local/runtime-path-check/activate.ps1`，用 `inspect_detector.cpp` 链接当前构建中的 `detector.cpp.obj`、`classifier.cpp.obj` 和同版 OpenCV，运行：

```powershell
outputs/armor_angle_diagnosis_20261029/inspect_detector.exe ../outputs/vscode/aim_hp_play_20261029_192040/cameras/0/vision/frame_g4_r8_n69368_t200.309000/image_raw.png src/rm_detector/model/RM_Armor_CNN_sim.onnx outputs/armor_angle_diagnosis_20261029
```
