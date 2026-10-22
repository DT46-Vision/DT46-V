# DT-46-KielasVison
梓喵-铭

<br><img src="DT46-vision.svg" alt="DT46_vision" width="200" height="200">

<br>我的爱如潮水，you 然，我已无法再与它 apart

## 🔗 相关资料
* **[技术文档-GitHub](https://github.com/DT46-Vision/RM_Kielas_Vision_doc.git)**
* **[CNN训练代码-GitHub](https://github.com/DT46-Vision/KIELAS_Armor_CNN_Train.git)**

---

## 📖 项目简介
本项目为 RoboMaster C++20 视觉自瞄系统，提供独立原生程序和 ROS 2 节点。硬件通信、图像采集、装甲板识别、三维追踪、弹道解算及调试显示均由 C++ 实现。独立程序的构建、虚拟相机接入及离线工具见 [standalone 说明](standalone/README.md)。ROS 节点与实机硬件尚未在本次 Windows 环境验证。

### 🧠 核心算法亮点
* **EKF 状态估计（C++ / Eigen）**：构建了 9 维状态向量 `[xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r]`，引入马氏距离（Mahalanobis Distance）剔除 PnP 野值，保障高速运动下的状态收敛。
* **空气动力学弹道解算**：基于平方空气阻力模型 (F = -kv^2) 的数值积分预建"距离 × 高度"查找表，运行期按弹速实时重建，配合动态云台/枪口外参补偿，实现高精度动态击打。
* **反小陀螺与状态机机制**：动态监测目标角速度与装甲板跳变（Armor Jump），利用多块装甲板的拓扑关系进行平滑切换，按时间累计判定进入/退出小陀螺，有效应对敌方车辆的高频原地旋转。
* **异步计算 + 快照渲染**：`rm_detector` 与 `rm_tracker` 都把重计算放在独立工作线程，追踪结果通过数据快照交给渲染，确保 EKF 高频迭代不被推流绘图阻塞。

---

## ⚙️ 系统架构与模块划分

1. **rm_detector (C++)** —— 装甲板检测与三维解算。传统视觉灯条提取 + CNN 数字分类（OpenCV DNN 加载 ONNX），输出相机系 PnP 坐标。节点名 `rm_detector`。
2. **rm_tracker (C++)** —— 目标追踪、位姿预测与发弹决策。相机系 → 世界系坐标变换、EKF、反小陀螺状态机、弹道解算，下发云台控制指令。节点名 `rm_tracker`。
3. **rm_serial & dm_imu (C++)** —— 硬件通信层。`dm_imu` 解析达妙 IMU 姿态（921600）；`rm_serial` 通过 CRC16 与下位机双向通讯（115200），转发目标颜色/弹速并下发云台解算结果。
4. **相机驱动层** —— 海康工业相机 (`hik_camera`)、迈德威视相机 (`mindvision_camera`)、USB/网络摄像头 (`usb_camera`)。
5. **rm_interfaces** —— 自定义消息定义（`ArmorsMsg` / `GimbalControl` / `Decision` 等），其余各包都依赖它，必须最先编译。
6. **rm_vision_bringup**（源码目录 `src/rm_DT46`）—— 只含 launch 的启动组合包，按兵种/场景把上述节点与参数文件组合起来。

### 📦 功能包索引

| 源码目录 | 包名 | 节点名（可执行名） | 作用 | 详细文档 |
| --- | --- | --- | --- | --- |
| `src/rm_interfaces` | `rm_interfaces` | 无（消息包） | 自定义消息定义 | [README](src/rm_interfaces/README.md) |
| `src/rm_detector` | `rm_detector` | `rm_detector` (`rm_detector_node`) | 装甲板检测 + PnP 测距 | [README](src/rm_detector/README.md) |
| `src/rm_tracker` | `rm_tracker` | `rm_tracker` (`rm_tracker_node`) | 追踪 + EKF + 弹道解算 | [README](src/rm_tracker/README.md) |
| `src/rm_serial` | `rm_serial` | `rm_serial` (`rm_serial_node`) | 下位机串口通信 | [README](src/rm_serial/README.md) |
| `src/rm_vision_ros2_dm_imu` | `dm_imu` | `dm_imu` (`dm_imu_node`) | 达妙 IMU 驱动 | [README](src/rm_vision_ros2_dm_imu/README.md) |
| `src/rm_vision_ros2_hik_camera` | `hik_camera` | `hik_camera` (`hik_camera_node`) | 海康相机驱动 | [README](src/rm_vision_ros2_hik_camera/README.md) |
| `src/rm_vision_ros2_mindvision_camera` | `mindvision_camera` | `mv_camera` (`mindvision_camera_node`) | 迈德威视相机驱动 | [README](src/rm_vision_ros2_mindvision_camera/README.md) |
| `src/rm_vision_ros2_usb_camera` | `usb_camera` | `usb_camera_node` | USB / 网络摄像头驱动 | [README](src/rm_vision_ros2_usb_camera/README.md) |
| `src/dt46_hardware` | `dt46_hardware` | 无（共享库） | C++ 串口、CRC 与 IMU 协议 | [硬件说明](src/rm_serial/README.md) |
| `src/dt46_tools` | `dt46_tools` | `record_images` / `calibrate_camera` / `make_sentry_variant` | 原生录制、标定和参数工具 | [工具说明](standalone/README.md) |
| `src/rm_DT46` | `rm_vision_bringup` | 无（launch 包） | 场景启动组合 | [README](src/rm_DT46/README.md) |

> ⚠️ 目录名与包名不一致的两个包：`src/rm_DT46` → `rm_vision_bringup`、`src/rm_vision_ros2_dm_imu` → `dm_imu`；`ros2 launch/run` 一律用**包名**。

---

## 📡 接口定义 (rm_interfaces)
系统内部的数据流转依赖自定义 ROS 2 消息类型（字段/单位详见 [rm_interfaces/README](src/rm_interfaces/README.md)）：
* `ArmorsMsg`: 包含多块装甲板当前帧的三维坐标 (`dx, dy, dz`，单位 **mm**) 与相对偏航角 (`yaw`，单位**度**)。
* `ArmorsDebugMsg`: 包含装甲板的四点灯条像素坐标、颜色及 ID 等调试信息。
* `GimbalControl`: 包含解算后的云台目标姿态 (`pitch, yaw`，单位**度**) 与开火许可标志 (`can_fire`，注意是 `int32`)。
* `Decision`: 协议里预留了比赛状态、血量、剩余时间等字段，**目前只有 `color`（目标颜色）与 `bullet_speed`（弹速）真正被下位机数据填充**。

关键话题链路（节点名 → 话题）：

```
相机驱动 ──/image_raw + /camera_info──▶ rm_detector ──/detector/armors_info──▶ rm_tracker
dm_imu  ──/imu/rpy(度)──────────────────────────────────────────────────────▶ rm_tracker
rm_serial ──/nav/decision(颜色/弹速)────────────────────────────────────────▶ rm_tracker
rm_tracker ──/tracker/gimbal_control──▶ rm_serial ──串口 0x5A 帧──▶ 下位机
```

---

## 🎯 装甲板类型定义

| 编号 | 含义             | 序号 |
|------|------------------|------|
| B1   | 蓝方1号 装甲板   | 0    |
| B2   | 蓝方2号 装甲板   | 1    |
| B3   | 蓝方3号 装甲板   | 2    |
| B4   | 蓝方4号 装甲板   | 3    |
| B5   | 蓝方5号 装甲板   | 4    |
| B7   | 蓝方哨兵 装甲板   | 5    |
| R1   | 红方1号 装甲板   | 6    |
| R2   | 红方2号 装甲板   | 7    |
| R3   | 红方3号 装甲板   | 8    |
| R4   | 红方4号 装甲板   | 9    |
| R5   | 红方5号 装甲板   | 10   |
| R7   | 红方哨兵 装甲板   | 11   |

> ⚠️ 上表是 `armor_id` 的完整编号约定，但**当前 `rm_detector` 只输出其中的 6 个 ID**：CNN 只有 3 个类别，`Armor::get_id()` 把「红/蓝 + 类别」映射为 6(R1) / 8(R3) / 11(R7) 与 0(B1) / 2(B3) / 5(B7)，其余组合返回 -1 并被丢弃。详见 [rm_detector/README](src/rm_detector/README.md)。

---

## 🛠️ 部署与环境配置

### 1. 环境依赖
* **系统环境**: Ubuntu 22.04
* **ROS 2 版本**: Humble
* **C++ 依赖库**: OpenCV（含 `dnn` 模块，用于加载 ONNX）、Eigen3、cv_bridge
  > `rm_tracker` 使用 C++ / Eigen 实现 EKF；`rm_detector` 用 `cv::dnn::readNetFromONNX` 推理，**不需要 ONNXRuntime**。

### 2. 编译与运行
克隆工作空间后，在根目录执行以下命令进行编译：
```bash
# 生成 compile_commands.json 方便代码补全，并使用符号链接安装以便本地调试
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install

# 引入环境变量
source install/setup.bash
```

### 3. 串口定向映射 (Udev 规则)
本系统使用多串口并发通信，需配置 udev 规则绑定软链接，防止设备号飘移。

> **推荐用仓库根目录的脚本自动完成**（免手抄 VID/PID/serial）：
> ```bash
> ./setup_serial.sh --discover                      # 先看插着的设备识别成什么
> ./setup_serial.sh                                 # 交互式选择 IMU / MCU 并安装规则
> ./setup_serial.sh --check                         # 校验软链接是否生效
> ./setup_serial.sh --imu /dev/ttyACM0 --mcu /dev/ttyACM1   # 也可直接指定，非交互
> ./setup_serial.sh --print                         # 只看将写入的规则，不动系统
> ./setup_serial.sh --uninstall                     # 删除规则
> ```
> 脚本会自动 sudo 提权、覆盖前备份旧规则。下面的手工步骤适用于需要自己核对属性的场合。

**获取串口信息**：
```bash
udevadm info -a -n /dev/ttyACM0 | grep -E '{idVendor}|{idProduct}|{serial}'
```
*(记录下输出结果中第一组的 `idVendor`、`idProduct` 和 `serial`)*

**编写 udev 规则**：
```bash
sudo nano /etc/udev/rules.d/99-fixed-serial.rules
```
填入以下规则（**务必用 `udevadm info` 查到的真实 `idVendor` / `idProduct` / `serial` 替换下面的示例值**，示例里的 `ffff` 是占位符）：
```bash
ACTION=="add", KERNEL=="ttyACM*", ATTRS{idVendor}=="ffff", ATTRS{idProduct}=="ffff", ATTRS{serial}=="2025021200", SYMLINK+="ttyPortIMU", MODE="0666"
ACTION=="add", KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="5909029513", SYMLINK+="ttyPortMCU", MODE="0666"
```

**重新加载规则**：
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
> ⚠️ **关键步骤**：配置完成后，请**务必手动将这两个串口的 USB 线从电脑上拔下并重新插入**。

---

### 4. 快捷启动配置 (Aliases)
建议在 `~/.bashrc` 中添加别名，实现一键拉起节点：
```bash
# nano ~/.bashrc
alias rmhiknav='ros2 launch rm_vision_bringup hik_nav.launch.xml'
alias rmhik='ros2 launch rm_vision_bringup hik.launch.xml'
alias rmmv='ros2 launch rm_vision_bringup mv.launch.xml'
```

---

### 5. 场景启动说明 (Launch Files)
`rm_vision_bringup` 提供了多种启动脚本以适配不同兵种与测试场景：
* **实车部署 (全节点启动)**:
  * `ros2 launch rm_vision_bringup hik.launch.xml`: 哨兵默认启动项 (海康相机 + 串口 + IMU + 检测 + 跟踪)。
  * `ros2 launch rm_vision_bringup mv.launch.xml`: 步兵默认启动项 (迈德威视相机 + 串口 + IMU + 检测 + 跟踪)。
* **兵种专用 (参数写死，不接受命令行覆盖)**:
  * `ros2 launch rm_vision_bringup hero.launch.xml`: 英雄 (海康相机 + 英雄标定/算法参数)。
  * `ros2 launch rm_vision_bringup sentry.launch.xml`: 哨兵 (海康相机 + 哨兵标定/算法参数)。
  * `ros2 launch rm_vision_bringup infantry.launch.xml`: 步兵 (迈德威视相机 + 步兵参数)。
* **离线/视觉独立测试 (无串口通信)**:
  * `hik_nav.launch.xml` / `mv_nav.launch.xml` / `usb_nav.launch.xml`: 仅启动相机、IMU及视觉算法层，用于纯视觉定位或录制数据集。
  * 注意 `mv_nav.launch.xml` **不启动 IMU**，而 `rm_tracker` 没有 `/imu/rpy` 就不会追踪；纯视觉联调追踪请用 `hik_nav` 或 `usb_nav`。
* **数据集回放**:
  * `ros2 launch rm_vision_bringup bag_record.launch.xml bag_file:=/path/to/bag`: 读取 rosbag 数据并跑通视觉流，便于算法调参。
