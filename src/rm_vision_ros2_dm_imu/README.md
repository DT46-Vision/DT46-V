# dm_imu

达妙 IMU 的 C++20 / `ament_cmake` 驱动。包名为 `dm_imu`，节点和参数根键也为 `dm_imu`，可执行文件为 `dm_imu_node`。串口与解析代码由 [硬件模块](../../project/modules/hardware/)维护，ROS 适配在 [src/node.cpp](src/node.cpp)。

## 数据与话题

后台线程非阻塞读取并解析全部完整帧，只保留最新样本与递增计数。100 Hz 定时器仅发布新样本，不重复发布缓存数据；高于发布频率的输入会被抽稀。发布时使用 ROS 当前时间。

| 话题 | 类型 | 开关及默认值 |
|---|---|---|
| `imu/rpy` | `geometry_msgs/msg/Vector3Stamped` | `publish_rpy: true`；默认单位度 |
| `imu/data` | `sensor_msgs/msg/Imu` | `publish_imu_data: false` |
| `imu/pose` | `geometry_msgs/msg/PoseStamped` | `publish_pose: false`；位置为原点 |

RPY 按 ZYX（yaw → pitch → roll）转换为四元数并归一化；非有限姿态使用单位四元数。Imu 的姿态协方差对角为 0.02，角速度和加速度协方差全部为 −1，表示没有这些测量。RPY 本身按原值或弧度转换输出。

`qos_reliable: true` 使用 Reliable / KeepLast(50)，否则使用 SensorDataQoS。Pose 使用队列深度 10 的默认可靠 QoS。话题为相对名称，默认命名空间下对应 `/imu/rpy` 等。

## 帧格式与校验

| 偏移 | 内容 |
|---|---|
| 0–1 | `0x55 0xAA` |
| 2 | 保留字节 |
| 3 | RID，接受 1、2、3 |
| 4、8、12 | 三个 float32 小端，依次作为 roll/pitch/yaw，单位度 |
| 16–17 | 校验值 uint16 小端 |
| 18 | 帧尾 `0x0A` |

为保持当前设备链路的字节行为，校验沿用既有实现：初值 `0xFFFF`、由 `0x1021` 生成的表，但每字节更新为 `(crc << 1) XOR table[((crc >> 8) XOR byte) & 255]`，截断至 16 位。这**不是标准 CCITT 的字节更新**。先校验前 16 字节，失败后兼容跳过两字节帧头的 14 字节范围；没有擅自改成标准校验值。若需纠正设备协议，必须先核对真实报文。

仍然不区分三种 RID 的业务意义，最新合法帧的三个浮点数都作为 RPY；不能据此认为驱动已支持分离加速度和角速度。错误帧会被丢弃并重新同步，缓存最多保留一个不完整帧。

## 参数与运行

参数唯一来源：[config/dm_imu_params.yaml](config/dm_imu_params.yaml)。

| 参数 | YAML 默认值 | 说明 |
|---|---|---|
| `port` / `baudrate` | `/dev/ttyPortIMU` / `921600` | 非阻塞 8N1，无流控 |
| `frame_id` | `imu_link` | 输出消息坐标系标签 |
| `publish_rpy_in_degree` | `true` | 设为 false 才转弧度；原 tracker 使用度 |
| `qos_reliable` | `true` | 发布 QoS |
| `verbose` | `false` | 姿态与解析统计日志 |

三个发布开关见上表。参数只在启动时读取，不提供串口热配置。打开失败明确报错退出；运行中读取异常会停止读线程并报告错误，没有自动重连。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dt46_hardware dm_imu --symlink-install
source install/setup.bash
ros2 launch dm_imu dm_imu.launch.xml
# 可选 RViz 显示
ros2 launch dm_imu dm_imu_rviz.launch.xml
```

RViz 入口使用同一份 `dm_imu_params.yaml`，并加载包内 `rviz/imu.rviz`。设备准备见 [MCU 串口说明](../rm_serial/README.md#构建与启动)。整车只使用一个 `/imu/rpy` 来源。

协议与姿态离线检查见 [非 ROS 工程验证](../../standalone/README.md#c-运行层迁移进行中)。本次未在 ROS 2 或真实 IMU 上验证节点构建、发布频率和串口通信。