# 装甲位姿离线训练

训练、评估和模型导出属于视觉项目，位于本目录；仿真项目仅生产数据。训练器通过 JSONL 清单读取图片和监督，不链接 MuJoCo、ROS 或在线 C++ 程序。数据格式、坐标、开放范围与负样本语义由[仿真采集工具](../../../tools/armor_dataset/README.md)维护；论文和上游来源见[参考资料](../../../.claude/reference/docs/vision/armor_pose/README.md)。本目录维护训练方法、运行命令和导出接口。

## 模型与边界

首版采用随机初始化的 MobileNetV3-Small 特征层、一个小型共享层及装甲存在性、贴纸、阵营、平移和 6D 旋转输出头。输入是实际 128×128 RGB 裁图及裁剪后的 K/D 特征；相机参数保留缩放对应的尺度信息。网络不读取位置真值、实例掩码、可见像素数或贴纸真值作为输入。分类词表只从训练集建立并保存在 checkpoint/导出元数据中；不能把缺少训练数据的类别视为已支持。

网络独立预测装甲存在性、贴纸、阵营、三维位置及旋转；深度、直线距离和偏角按[已确认的固定计算契约](../../../tools/armor_dataset/README.md#输出和固定计算)从同一份位姿取得，没有额外的深度、距离或偏角输出头。报告中的这些派生项用于检查最终接口结果，不代表新增训练目标。

唯一的平移头内部用 X/Z、Y/Z 与 log(Z) 表示位置，再解码为 (X,Y,Z)；这仍是三维位置的学习估计，取指数只是内部解码，不是根据灯条尺寸独立测距。深度直接取该位置的 Z 分量。旋转输出依次为旋转矩阵第一、第二列。输出头和损失见 [model.py](model.py)。当前几何只覆盖清单中已核验的小装甲；同一贴纸若对应不同几何，训练器明确拒绝，后续需增加独立板型监督。

优化器为 AdamW；默认 CUDA + FP16 AMP，损失与几何计算保留 FP32，裁剪梯度前先反缩放。缺少 CUDA 会报错，用户可显式选择 CPU。当前超参数是启动基线，未做最优搜索。骨干包含自身的 SE 模块，没有额外添加大型注意力或稀疏化；训练不做会破坏姿态标签的随意翻转、旋转或透视增强，已有成像随机化保留在源数据中。

训练器严格沿用 train/validation/test 和 scene_group，拒绝跨集合的场景组与重复图片。监督掩码分别控制位姿、贴纸和阵营损失；负样本只训练装甲存在性。验证集选择最佳权重，测试集不参与每轮训练和选权。想限制角度、类别或负样本，应先按采集工具的重选流程导出新清单，再将它传给训练器。

## 本机环境

以下命令在视觉仓库根目录运行。环境独立放在已被 Git 忽略的 `.cache/armor-training-env`；创建后无需反复安装。PyTorch 包来自官方 CUDA 12.8 索引，其余直接依赖来自官方 PyPI，并锁定版本。

```powershell
python -m venv .cache/armor-training-env
.cache/armor-training-env/Scripts/python.exe -m pip install -r tools/armor_pose_training/requirements-gpu.txt
.cache/armor-training-env/Scripts/python.exe -m pip install -r tools/armor_pose_training/requirements.txt
.cache/armor-training-env/Scripts/python.exe -m unittest tools.armor_pose_training.test_training -v
```

CUDA 框架包含运行所需的 CUDA 库；是否能实际训练以 GPU 前向、反向和更新检查为准，不能只看显卡被识别。本机验证环境与结果由[首次运行记录](../../outputs/armor_pose_training_20261030/report.md)维护。Linux/ROS 安装不属于本次 Windows 环境验证。

## 训练与恢复

```powershell
.cache/armor-training-env/Scripts/python.exe -m tools.armor_pose_training.train --manifest ../outputs/armor_batch_20261030/dataset_10000/training/training.jsonl --output .cache/armor-pose-runs/my_run --epochs 20 --batch-size 128
# --epochs 表示总轮数；断点续训沿用原优化器、AMP、batch 和随机流状态。
.cache/armor-training-env/Scripts/python.exe -m tools.armor_pose_training.train --manifest ../outputs/armor_batch_20261030/dataset_10000/training/training.jsonl --output .cache/armor-pose-runs/my_run --resume .cache/armor-pose-runs/my_run/last.pt --epochs 30 --batch-size 128
```

新训练要求输出目录不存在。数据和权重留本机，不复制进源码；`.cache/armor-pose-runs` 是上述示例的实际保存位置，用户也可指定其他输出目录。`last.pt` 保存最后一轮及优化器、AMP scaler 和随机状态；`best.pt` 按完整验证集的固定加权损失选择，不是按测试误差选择。`history.jsonl` 保留每轮训练、验证、耗时和显存，`environment.json` 保存命令、版本、类别和清单 SHA-256；恢复时额外记录本次环境。

`--workers` 默认 0，便于 Windows 首次运行；可在本机计时后调整。`--no-amp` 用于 FP32 对照；恢复要求 AMP 和 batch 与原训练一致。学习率/权重衰减选项只初始化新训练，恢复使用 checkpoint 的优化器状态。随机种子有记录，但不承诺不同硬件/驱动下逐位一致。

## 评估与 CPU 导出

```powershell
.cache/armor-training-env/Scripts/python.exe -m tools.armor_pose_training.evaluate --manifest ../outputs/armor_batch_20261030/dataset_10000/training/training.jsonl --checkpoint .cache/armor-pose-runs/my_run/best.pt --split test --output .cache/armor-pose-runs/my_run/test_metrics.json
.cache/armor-training-env/Scripts/python.exe -m tools.armor_pose_training.export --manifest ../outputs/armor_batch_20261030/dataset_10000/training/training.jsonl --checkpoint .cache/armor-pose-runs/my_run/best.pt --output .cache/armor-pose-runs/my_run/armor_pose.onnx
```

评估包含正负样本的误检/漏检、受监督类别准确率、有符号深度偏差、深度标准差、绝对误差分位数、平移 RMSE 和旋转角误差。位姿统计包含全部受监督正样本，不因存在性得分低而移除失败样本；退化旋转另计数量。深度与斜视分层独立报告。训练时验证采用 FP32 前向，以便与导出后的 CPU 模型对应。

固定首版权重的完整 CPU 测试、逐图真值对照和错误样本见[测试集输出核对](../../outputs/armor_pose_outputs_20261030/report.md)。该记录区分独立预测与位姿派生项；不使用测试集重新训练或选择权重。

ONNX 导出后实际检查图模型，并用最多 32 张验证裁图、batch=1/2/4 比较 PyTorch CPU 和 ONNX Runtime CPU 的全部输出。模型旁的同名 JSON 是该权重的部署接口快照，保存 RGB 范围、K/D 特征顺序、单位、旋转排列、类别、模型哈希及一致性检查结果。分类输出为 logits；存在性分数尚未做概率校准，也不等于位姿置信度或测量协方差。

导出计时仅覆盖预处理完成后的 ORT CPU 前向，不包括候选提取、原图裁剪/缩放、输入准备、几何解码和 IO，因此不能作为完整视觉单拍耗时。模型已接入原生候选、识别、定位及车辆 EKF，部署契约见 [standalone](../../standalone/README.md#管线接口与坐标)，完整时延和精度见[模型运行验证](../../outputs/armor_model_runtime_20261030/report.md)。K/D 作为输入也不自动保证未训练标定的泛化。

原生 OpenCV 部署使用同一导出命令加 `--fixed-batch`，输出另存，保留训练用动态批量文件：

```powershell
.cache/armor-training-env/Scripts/python.exe -m tools.armor_pose_training.export --manifest ../outputs/armor_batch_20261030/dataset_10000/training/training.jsonl --checkpoint .cache/armor-pose-runs/bootstrap_20261030/best.pt --output .cache/armor-pose-runs/bootstrap_20261030/armor_pose_opencv.onnx --fixed-batch
```

此选项仅固定 batch=1，导出检查与计时只跑 batch=1；不修改权重、输入内容、输出头或损失。部署时 ONNX 与同名 JSON 一起使用，原生路径由 [armor_model.json](../../standalone/armor_model.json)维护。更换 checkpoint 后先重新核对独立测试集与整帧结果。
