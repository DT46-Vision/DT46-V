# rm_serial

下位机（MCU/电控）串口通信节点（Python / `ament_python`）。负责：

- 向上：解析下位机上行帧（20 字节，`0xA5` 帧头 + CRC16），发布为 `/nav/decision`（颜色、弹速）与可选的 `/imu/rpy`。
- 向下：订阅 `/tracker/gimbal_control`，以 100 Hz 打包下行帧（`0x5A` 帧头 + CRC16）发给下位机。

## 节点与可执行文件

| 节点名（ROS 图中的名字） | 可执行文件 | 作用 |
| --- | --- | --- |
| `rm_serial` | `rm_serial_node` | 双向串口收发 + CRC 校验 + 断线重连 |

- `rm_serial.node:SerialNode` 里构造时写的是 `"serial_node"`，但 launch 用 `name="rm_serial"` 覆盖了节点名，所以 ROS 图中看到的是 **`rm_serial`**，YAML 根键也必须是 `rm_serial:`（`config/rm_serial_params.yaml` 一致）。
- `ros2 run rm_serial rm_serial_node` 不加 `--ros-args -r __node:=rm_serial` 时节点名会是 `serial_node`，此时 YAML 参数不会被加载。

## 话题接口

| 方向 | 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- | --- |
| 发布 | `/nav/decision` | `rm_interfaces/msg/Decision` | BEST_EFFORT / KEEP_LAST(10) / VOLATILE | 填 `color`（下位机 `detect_color`）与 `bullet_speed`；`frame_id = serial_receive_frame` |
| 发布 | `/imu/rpy` | `geometry_msgs/msg/Vector3Stamped` | 同上 | **仅 `pub_rpy: true` 时发布**；`frame_id = imu_link`，`x/y/z` 即下位机 `roll/pitch/yaw`（原样透传，单位由下位机决定） |
| 订阅 | `/tracker/gimbal_control` | `rm_interfaces/msg/GimbalControl` | `qos_profile_sensor_data`（BestEffort） | 云台 yaw/pitch/can_fire，缓存后由发送线程周期性下发 |

## 串口协议

### 上行（下位机 → 本节点），帧长 20 字节

| 偏移 | 长度 | 内容 |
| --- | --- | --- |
| 0 | 1 | 帧头 `0xA5` |
| 1 | 1 | `detect_color`（0 = 红，1 = 蓝） |
| 2 | 4 | `roll`（float32 小端） |
| 6 | 4 | `pitch`（float32 小端） |
| 10 | 4 | `yaw`（float32 小端） |
| 14 | 4 | `bullet_speed`（float32 小端） |
| 18 | 2 | CRC16 小端（校验范围为前 18 字节，初值与查表见 `modules/crc.py` / `crc_table.py`） |

解析用 `struct.unpack("<BBffff", payload)`。CRC 失败会丢弃该帧并计数，前 5 次失败会打印原始报文与两端 CRC 值。

### 下行（本节点 → 下位机），帧长 15 字节

| 偏移 | 长度 | 内容 |
| --- | --- | --- |
| 0 | 1 | 帧头 `0x5A` |
| 1 | 4 | `yaw`（float32 小端，度） |
| 5 | 4 | `pitch`（float32 小端，度） |
| 9 | 4 | `can_fire`（int32 小端，0/1） |
| 13 | 2 | CRC16 小端 |

发送节奏由独立线程 `send_data_loop` 控制：每 5 ms 发一次（约 100 Hz），即使没有新的 `/tracker/gimbal_control` 也会重复发送上一帧值。

## 线程模型

| 线程 | 作用 |
| --- | --- |
| ROS spin（主线程） | 接收 `/tracker/gimbal_control`，加锁写入 `send_datas` |
| `receive_thread` | 非阻塞读串口 → 环形缓冲找帧头 → 长度/CRC 校验 → 解析 → 发布 |
| `send_thread` | 每 5 ms 组帧下发 |

串口异常时由接收线程触发 `reopen_port()`：关闭并每 1 s 重试打开，重连期间发送线程直接跳过（不会向已关闭的串口写入）。

## 参数

文件：`config/rm_serial_params.yaml`（根键 `rm_serial:`）。

| 参数 | 类型 | 默认值（YAML） | 含义 |
| --- | --- | --- | --- |
| `port_name` | string | `/dev/ttyPortMCU` | 串口设备（udev 软链接，见下文） |
| `baudrate` | int | `115200` | 波特率 |
| `timeout` | double | `1.0` | 读超时（s） |
| `write_timeout` | double | `1.0` | 写超时（s） |
| `flow_control` | string | `none` | **读取了但没有传给 `serial.Serial()`，当前无效** |
| `parity` | string | `none` | **同上，当前无效** |
| `stop_bits` | string | `1` | **同上，当前无效** |
| `serial_receive_header` | int | `0xA5` | 上行帧头，**会被代码里硬编码的 0xA5 覆盖，当前无效** |
| `serial_send_header` | int | `0x5A` | 下行帧头，**会被硬编码的 0x5A 覆盖，当前无效** |
| `pub_rpy` | bool | `false` | 是否把下位机 RPY 转发到 `/imu/rpy` |

## 使用方式

### 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/DT46-V
colcon build --packages-select rm_interfaces rm_serial --symlink-install
source install/setup.bash
```

### 启动

```bash
ros2 launch rm_serial rm_serial.launch.py
```

该 launch 没有暴露 launch argument，参数固定取 `config/rm_serial_params.yaml`。整机场景下由 `rm_vision_bringup` 的 `hik.launch.py` / `mv.launch.py` / `hero.launch.py` / `sentry.launch.py` 启动；其中 `hik` / `mv` 支持 `serial_params_file:=...` 覆盖参数文件，`hero` / `sentry` 的参数路径写死在 launch 里（详见 `rm_vision_bringup` README）。

```bash
# 直接运行（注意节点名，见上文）
ros2 run rm_serial rm_serial_node --ros-args -r __node:=rm_serial \
  --params-file install/rm_serial/share/rm_serial/config/rm_serial_params.yaml
```

### 串口设备准备（udev）

参数默认使用 udev 软链接，必须先建立规则，否则节点会因为打不开设备而直接抛异常退出：

1. 查设备信息（插上设备后）：
   ```bash
   udevadm info -a -n /dev/ttyACM0 | grep -E '{idVendor}|{idProduct}|{serial}'
   ```
2. 新建 `/etc/udev/rules.d/99-fixed-serial.rules`（serial 换成实测值）：

   ```bash
   ACTION=="add", KERNEL=="ttyACM*", ATTRS{idVendor}=="ffff", ATTRS{idProduct}=="ffff", ATTRS{serial}=="2025021200", SYMLINK+="ttyPortIMU", MODE="0666"
   ACTION=="add", KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", ATTRS{serial}=="5909029513", SYMLINK+="ttyPortMCU", MODE="0666"
   ```
3. 生效并**重新插拔 USB**：
   ```bash
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
4. 验证：`ls -l /dev/ttyPortMCU /dev/ttyPortIMU`

> 注意：根 README 里写的 `idVendor=ffff / idProduct=ffff` 是占位值，必须换成自己车上的真实值。

### 调试手段

```bash
ros2 topic echo /nav/decision          # 看下位机是否在上行（颜色/弹速）
ros2 topic hz /nav/decision            # 上行帧率（受下位机发送频率限制）
ros2 topic echo /tracker/gimbal_control
ros2 topic hz /tracker/gimbal_control
# 无硬件时手动灌数据测试下行
ros2 topic pub -r 100 /tracker/gimbal_control rm_interfaces/msg/GimbalControl "{yaw: 1.0, pitch: 2.0, can_fire: 1}"
```

## 依赖与前置条件

- 运行依赖：`rclpy`、`python3-serial`（`exec_depend`）、`rm_interfaces`、`geometry_msgs`。
- 硬件：一条与下位机相连的串口（默认 115200 8N1），以及上文的 udev 规则。
- 需要下位机按上面的帧格式发上行帧；帧头/长度/CRC 任一不符都会被丢弃。

## 注意事项 / 已知问题

1. **节点名依赖 launch 覆盖**：直接 `ros2 run` 时节点名是 `serial_node`，参数文件不会生效（YAML 根键是 `rm_serial`）。要么加 `-r __node:=rm_serial`，要么改 YAML 根键。
2. **`flow_control` / `parity` / `stop_bits` / 两个帧头参数当前无效**：前者没有传给 `serial.Serial()`，后者在 `__init__` 里被硬编码的 `0xA5`/`0x5A` 覆盖。想改协议要先改代码。
3. **上行帧长写死 20 字节**：`packet_length = 20` 是常量，下位机改协议长度时这里必须同步。
4. **`Decision` 只填了两个字段**：血量、剩余时间、比赛进程等未解析（消息里定义了，但没有对应上行数据），见 `rm_interfaces` README。
5. **`/imu/rpy` 的潜在冲突**：`pub_rpy: true` 时本节点也会发 `/imu/rpy`，与 `dm_imu` 话题同名且单位不一定一致（`dm_imu` 默认发"度"）。整车当前用 `dm_imu` 供姿态，`pub_rpy` 保持 false。
6. **发送是"重复发上一帧"**：100 Hz 定时发送，不判断数据新旧；`/tracker/gimbal_control` 断流时下位机会持续收到最后一次的 yaw/pitch（以及当时的 `can_fire`）。
7. **CRC 失败只打印前 5 次**：之后静默计数，排查线缆/波特率问题时可临时把 `crc_fail_count` 的打印上限调大。
8. **重连只在接收异常时触发**：如果只是发送失败（例如拔线后 `write` 抛错），代码只打印错误不重连，需要重启节点或等待下一次接收异常。
