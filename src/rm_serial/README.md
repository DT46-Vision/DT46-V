# rm_serial

下位机串口节点，使用 C++20 / `ament_cmake`。协议与系统串口实现维护在 [硬件模块](../../project/modules/hardware/)，ROS 适配在 [src/node.cpp](src/node.cpp)。

节点可执行文件为 `rm_serial_node`，默认节点名 `serial_node`；[XML 启动入口](launch/rm_serial.launch.xml) 将其命名为 `rm_serial`，与参数文件根键一致。

## 接口与协议

| 方向 | 接口 | 约定 |
|---|---|---|
| 发布 | `/nav/decision`，`rm_interfaces/msg/Decision` | BestEffort / KeepLast(10)；仅填颜色与弹速，frame 为 `serial_receive_frame` |
| 可选发布 | `/imu/rpy`，`geometry_msgs/msg/Vector3Stamped` | 同上；`pub_rpy: true` 时透传 roll/pitch/yaw，frame 为 `imu_link`，单位由下位机决定 |
| 订阅 | `tracker/gimbal_control`，`rm_interfaces/msg/GimbalControl` | SensorDataQoS；缓存 yaw/pitch/can_fire |

上行固定 20 字节，小端：

| 偏移 | 内容 |
|---|---|
| 0 | `0xA5` |
| 1 | 颜色 uint8，0 红 / 1 蓝 |
| 2、6、10、14 | roll、pitch、yaw、bullet_speed，各 float32 |
| 18 | 前 18 字节的 CRC16，uint16 小端 |

下行固定 15 字节，小端：

| 偏移 | 内容 |
|---|---|
| 0 | `0x5A` |
| 1、5 | yaw、pitch，各 float32，单位度 |
| 9 | can_fire，int32 |
| 13 | 前 13 字节的 CRC16，uint16 小端 |

MCU CRC 初值 `0xFFFF`，反射多项式 `0x8408`，无末尾异或。编码显式处理小端与 IEEE754 float32，不依赖结构体打包或主机字节序。

## 调度与参数

单个 I/O 线程拥有串口，循环非阻塞接收，并在上一次发送完成后间隔 5 ms 再发；理论上限约 200 Hz，实际受 I/O 与调度影响。接收和发送异常均在该线程关闭端口，每秒重试连接，重连成功前不写数据。退出时请求停止并等待线程结束。CRC 错误会丢弃并计数，前五次错误期间输出计数；解析器最多保留一个不完整帧，坏帧后继续寻找帧头。

参数文件：[config/rm_serial_params.yaml](config/rm_serial_params.yaml)。参数在启动时读取。

| 参数 | YAML 默认值 | 行为 |
|---|---|---|
| `port_name` / `baudrate` | `/dev/ttyPortMCU` / `115200` | 串口设备与速率 |
| `timeout` | `1.0` | 保留配置兼容；接收采用非阻塞轮询 |
| `write_timeout` | `1.0` | 写入超时秒数，范围 0–60 |
| `flow_control` / `parity` / `stop_bits` | `none` / `none` / `1` | 当前协议仅支持 8N1、无流控；其他值明确拒绝 |
| `serial_receive_header` / `serial_send_header` | `0xA5` / `0x5A` | 固定协议，其他值明确拒绝 |
| `pub_rpy` | `false` | 显式开启时发布下位机 RPY，不再被硬编码覆盖 |

停止接收新控制消息时仍重复最后一组指令，包括 `can_fire`，没有新增超时归零策略。整车使用 DM IMU 供姿态时保持 `pub_rpy: false`，避免同名话题有两个来源。

## 构建与启动

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dt46_hardware rm_interfaces rm_serial --symlink-install
source install/setup.bash
ros2 launch rm_serial rm_serial.launch.xml
```

直接运行时使用 `ros2 run rm_serial rm_serial_node --ros-args -r __node:=rm_serial --params-file install/rm_serial/share/rm_serial/config/rm_serial_params.yaml`。串口设备的发现与 udev 设置由根目录 [setup_serial.sh](../../setup_serial.sh) 提供；设备 ID 应从实机读取。

协议的离线对照检查见 [原生构建与测试](../../standalone/README.md#构建)。ROS 2 节点编译、真实端口收发和断线重连尚未在本次 Windows 环境验证。
