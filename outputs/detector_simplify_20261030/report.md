# 检测器配对修复与精简验证

基线 `cafc5f4`，开发分支 `codex/vision-detector-simplify`，继承 `codex/relative-aim-restart`。本轮查阅“基于实车输入重做自瞄与车心估计”和“梳理自瞄仿真时序与开火门控”，沿用实车输入限制；仅修改视觉检测器、测试与对应文档，没有修改仿真核心或接入额外传感器。旧分支与历史失败证据保留。

## 假设、方法和结果

待验证假设：首个候选失败后继续检查，能够恢复已被数字轮廓遮挡的真灯配对；接受后跳过整个灯对区间，可以防止灯条复用与交叉。使用相同图像、模型、阈值，要求恢复真实灯对而非任意多报一块板，并用相邻板、颜色、几何、分类、ROI 与空帧反例检验。通过这些检查后结束本轮，不将有限回归推广为全视角精度或误检率保证。

保存帧源为仿真会话 `outputs/vscode/aim_hp_play_20261029_192040/cameras/0/vision/frame_g4_r8_n69368_t200.309000/image_raw.png`，问题来源见[旧诊断](../armor_angle_diagnosis_20261029/report.md)。入库夹具 `tests/data/detector_digit_obstruction.png` 为原图矩形 `(x=550,y=540,w=650,h=340)` 的无缩放裁剪，保留全部四个候选；未改变颜色、像素值、检测参数或 ONNX。

- 修改前新回归实际失败：`lights=4 armors=0`，见 [before-test.txt](before-test.txt)。修改后四条候选仍在，准确输出原排序第 1、3 条组成的蓝色 3 号板。
- 交叉反例中，第二组灯对单独分类也能通过，但已接受区间内部的灯不能再次参与配对。两张夹具左右拼接仍输出两块独立装甲。ROI 恢复原图坐标、打乱候选顺序、严格宽度限制、红色拒绝/双色接受、空白分类为反例，以及少灯/无灯/空图像缓存清理全部通过。
- 原生 CTest 11/11，通过记录见 [ctest.txt](ctest.txt)。随后补强测试，新增空白分类拒绝与公平计时路径，`detector_pairing` 再次通过。首次全套运行的 `native_pipeline` 因 PATH 未包含构建中的 `standalone/dt46_aim.dll` 而报 `0xc0000135`；补齐本进程 DLL 路径后通过，没有改代码绕过失败。

## 实际修改

1. 检测器由“首个同色平行候选失败即放弃”改为“首个几何与分类均通过的候选才接受”。接受后直接移动左索引到右端，删除 `used` 位图，保守拒绝嵌套、交叉与重复配对。最坏仍为二次候选遍历，CNN 只在几何通过后运行，没有引入优化器或迭代求解。
2. 灯条几何筛选与颜色采样合为一次轮廓遍历，删除旋转矩形中间容器；直接复用灯条/装甲结果容器，去掉入口的重复赋值。删除本包未使用的斜率函数，高低灯分支改为 min/max，颜色/类别映射改为原值查表。两份运行源码合计净删 66 行。
3. 每轮配对统一清空装甲缓存和分类预览，替代每次间距拒绝时重建占位图；少于两灯或空输入不会保留旧装甲。分类预览显示本轮最后一次实际分类，不再被后续几何拒绝抹掉。
4. 增加 `detector_pairing` CTest 与约 121 KB 的像素夹具；算法说明只维护在 `src/rm_detector/README.md`，原生测试入口同步 `standalone/README.md`。默认 `build/native/bin` 的 EXE/DLL 更新为本次验证构建，替换前产物留在本目录 `previous-native-bin/`，摘要见 `identities.json`。

## 耗时与闭环

GCC Release、现有 OpenCV CPU DNN；每项预热 20 次、测量 300 次，单位 ms，未固定 CPU 频率或隔离系统负载。基线检测器取自 `cafc5f4`，两版使用相同测试程序、分类器对象文件、图像和模型。见 [baseline-benchmark.txt](baseline-benchmark.txt)、[final-benchmark.txt](final-benchmark.txt)。

| 比较对象 | 基线中位 / P95 | 修改后中位 / P95 | 输出与解释 |
|---|---:|---:|---|
| 650×340 保存帧完整检测 | 0.1527 / 0.1964 | 0.5862 / 0.8156 | 0 块 → 1 块，恢复分类增加工作量，不是提速 |
| 同一对真灯的配对及分类 | 0.4593 / 0.6448 | 0.4313 / 0.5763 | 两版均输出 1 块，只支持本次开销相近，不能据此宣称稳定性能提升 |

沿用已有 `run-cases.ps1`，三场各 20 s，真实渲染 RGB → 检测/PnP → tracker → 发弹/扣血；使用已有 350 HP、只开启装甲扣血的自动化副本。原始结果在 `../aim_model_validation_20261029/live-20261030-002611/`，汇总见 [live.txt](live.txt)。

| 场景 | 帧数 | 发弹数 | 靶车最终 HP | 无效许可 | 进程退出 |
|---|---:|---:|---:|---:|---|
| 静止 | 1376 | 18 | 0 | 0 | 主机/视觉均 0 |
| 原地匀速旋转 | 1463 | 18 | 0 | 0 | 主机/视觉均 0 |
| 正弦平移加旋转 | 1529 | 18 | 0 | 0 | 主机/视觉均 0 |

这些结果证明三场能发弹扣血；未统计独立弹丸命中率或车心/PnP 精度。目标被击毁后大量帧处于 acquiring，不能用全程 TRACKING 比例比较跟踪质量。

## 复现与边界

从视觉根目录执行，现有本机构建目录配置见 `standalone/README.md`：

```powershell
. ../.local/runtime-path-check/activate.ps1
$env:PATH=(Join-Path (Get-Location) 'build/aim-validation-full/standalone')+';'+$env:PATH
cmake --build build/aim-validation-full --target dt46_detector_check dt46_vision dt46_core_check dt46_pipeline_check --parallel 4
ctest --test-dir build/aim-validation-full --output-on-failure
build/aim-validation-full/dt46_detector_check.exe tests/data/detector_digit_obstruction.png src/rm_detector/model/RM_Armor_CNN_sim.onnx --benchmark
outputs/aim_model_validation_20261029/run-cases.ps1 -Cases stationary_off,stationary_constant,sine_sine -Seconds 20
```

偏离原方案：无。未完成：白字成为假灯的颜色判据仍未收紧；分类器误报可能使首个错误灯对提前通过；投影区间重叠的真实多板不能同时输出。原始 PnP 的系统偏差与跨视角一致性仍见[静止诊断的未完成项](../pnp_static_diagnosis_20261029/report.md#原始-pnp-检查及未完成事项)，本轮没有通过调参掩盖它们。未验证 ROS、实车、全部遮挡/倾斜场景或硬实时预算。
