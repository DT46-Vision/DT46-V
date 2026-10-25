# dm_imu

达妙（DM）IMU 串口驱动（Python / `ament_python`）。后台线程以非阻塞方式读串口、按帧解析并只保留最新一帧姿态，主线程用 100 Hz 定时器发布为 ROS 话题。

> 包名是 `dm_imu`，源码目录名是 `rm_vision_ros2_dm_imu`，编译/启动命令用 **`dm_imu`**。

## 节点与可执行文件

| 节点名（ROS 图中的名字） | 可执行文件 | 作用 |
| --- | --- | --- |
| `dm_imu` | `dm_imu_node` | 读取 IMU 串口，发布 RPY / Imu / Pose 三种姿态话题 |

- 节点名硬编码为 `DmImuNode() : Node('dm_imu')`，YAML 根键必须是 `dm_imu:`。
- 没有参数回调，所有参数都是启动时读取，改参数需要重启节点。

## 话题接口

| 方向 | 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- | --- |
| 发布 | `/imu/rpy` | `geometry_msgs/msg/Vector3Stamped` | `qos_reliable` 决定（见下） | `x/y/z = roll/pitch/yaw`，默认**单位是度**；`publish_rpy` 控制开关（默认开） |
| 发布 | `/imu/data` | `sensor_msgs/msg/Imu` | 同上 | 只有姿态（四元数），角速度/线加速度协方差置 −1 表示无效；`publish_imu_data` 控制开关（**默认关**） |
| 发布 | `/imu/pose` | `geometry_msgs/msg/PoseStamped` | 队列 10 | 位置恒为原点 + IMU 姿态，方便 RViz 直接看；`publish_pose` 控制开关（**默认关**） |

QoS 由 `qos_reliable` 决定：

- `true`（`dm_imu_params.yaml` 里的值）：RELIABLE / KEEP_LAST(50) / VOLATILE。
- `false`：`qos_profile_sensor_data`（BestEffort / KEEP_LAST(5)）。

三个话题名都是相对名，节点名保持 `dm_imu` 时对应 `/imu/rpy`、`/imu/data`、`/imu/pose`。`header.frame_id` 取参数 `frame_id`（默认 `imu_link`）。

## 数据流

1. `DM_Serial` 打开串口（非阻塞，`timeout=0`），`start_reader()` 起一个后台读线程，循环解析并保存"最新一帧 + 本地时间戳 + 计数"。
2. 节点 100 Hz 定时器（`0.01 s`）调 `get_latest()`：拿到新帧才发布，时间戳与上一帧相同则跳过（去重）。
3. RPY（度）→ 弧度 → `euler_rpy_to_quat()` 转四元数（ZYX 内旋，即 yaw→pitch→roll）；四元数做有限性检查与归一化，异常时退回单位四元数 `(0,0,0,1)`。
4. `/imu/rpy` 按 `publish_rpy_in_degree` 选择发度或弧度。
5. 另有 2 s 一次的统计定时器（`DM_Serial.get_stats()` 存在时才打印，需 `verbose: true`）。

## 串口协议（达妙 IMU 上行帧）

帧长固定 **19 字节**，小端：

| 偏移 | 长度 | 内容 |
| --- | --- | --- |
| 0 | 2 | 帧头 `0x55 0xAA` |
| 2 | 1 | 保留/功能字节（解析时跳过） |
| 3 | 1 | `RID`，合法值 `{0x01, 0x02, 0x03}` |
| 4 | 4 | float32（第 1 个，节点当作 roll） |
| 8 | 4 | float32（第 2 个，pitch） |
| 12 | 4 | float32（第 3 个，yaw） |
| 16 | 2 | CRC16 小端 |
| 18 | 1 | 帧尾 `0x0A` |

- CRC 默认按 `frame[0:16]`（含帧头）计算，校验失败会自动再试 `frame[2:16]`（不含帧头）以兼容两种下位机实现。
- 解析时会丢弃帧头不匹配、帧尾不符、RID 非法、CRC 错误的帧，并分别计数（`cnt_ok / cnt_crc / cnt_short / cnt_nohdr`）。
- 节点只取"最新一帧"的 3 个 float 当作 roll/pitch/yaw，**不区分 RID**；如果下位机用不同 RID 分别发加速度/角速度/欧拉角，这里会混用，需要改 `node.py`。

## 参数

文件：`config/dm_imu_params.yaml`（根键 `dm_imu:`）。

| 参数 | 类型 | 代码默认值 | YAML 值 | 含义 |
| --- | --- | --- | --- | --- |
| `port` | string | `/dev/ttyACM0` | `/dev/ttyPortIMU` | 串口设备（udev 软链接） |
| `baudrate` | int | `921600` | `921600` | 波特率；非法值会回退 921600 并告警 |
| `frame_id` | string | `imu_link` | `imu_link` | 消息 `header.frame_id` |
| `publish_rpy` | bool | `true` | `true` | 是否发布 `/imu/rpy`（**追踪节点依赖它**） |
| `publish_imu_data` | bool | `false` | `false` | 是否发布 `/imu/data` |
| `publish_pose` | bool | `false` | `false` | 是否发布 `/imu/pose` |
| `publish_rpy_in_degree` | bool | `true` | `true` | true 发度、false 发弧度（`rm_tracker` 要求**度**） |
| `qos_reliable` | bool | `false` | `true` | true = RELIABLE/KEEP_LAST(50)，false = SensorDataQoS |
| `verbose` | bool | `false` | `false` | 终端打印姿态、统计与告警（排查时打开） |

## 使用方式

### 编译

```bash
source /opt/ros/humble/setup.bash
cd ~/DT46-V
colcon build --packages-select dm_imu --symlink-install
source install/setup.bash
```

### 启动

```bash
# 只启动 IMU 节点（推荐）
ros2 launch dm_imu dm_imu.launch.py

# 直接运行
ros2 run dm_imu dm_imu_node --ros-args --params-file install/dm_imu/share/dm_imu/config/dm_imu_params.yaml
```

整机场景由 `rm_vision_bringup` 的 `hik.launch.py` / `mv.launch.py` / `hero.launch.py` / `sentry.launch.py` / `usb_nav.launch.py` 启动；其中 `hik` / `mv` / `hik_nav` / `usb_nav` 支持 `dm_imu_params_file:=...` 覆盖参数文件，`hero` / `sentry` 的路径写死在 launch 里（详见 `rm_vision_bringup` README）。

### 验证

```bash
ros2 topic list | grep imu
ros2 topic echo /imu/rpy        # 默认单位是度
ros2 topic hz /imu/rpy          # 正常情况下接近 100 Hz（受 IMU 实际上报率限制）
```

### 串口设备准备（udev）

参数默认使用 `/dev/ttyPortIMU` 软链接，规则见 `rm_serial` README 的"串口设备准备"一节（IMU 与 MCU 两条规则写在同一个文件里）。没有规则时可临时改 `port` 参数指向 `/dev/ttyACM*`。

## 依赖与前置条件

- 运行依赖：`rclpy`、`sensor_msgs`、`geometry_msgs`、`python3-serial`（`exec_depend`，也可以用 pip 的 `pyserial`，此时删掉 package.xml 里的 apt 依赖并在 setup.py 里打开注释）。
- 硬件：达妙 IMU，USB 转串口，默认 921600 波特率。
- udev 软链接 `/dev/ttyPortIMU`（或把 `port` 改成实际设备）。

## 注意事项 / 已知问题

1. **`dm_imu_rviz.launch.py` 目前是坏的**：它加载的是 `config/params.yaml`，而实际文件叫 `config/dm_imu_params.yaml`（`setup.py` 也只安装后者）。用它会拿不到参数（退回代码默认值 `/dev/ttyACM0`），大概率打不开串口。可视化建议这样用：

   ```bash
   ros2 launch dm_imu dm_imu.launch.py &
   rviz2 -d install/dm_imu/share/dm_imu/rviz/imu.rviz
   ```

   （`rviz/imu.rviz` 本身存在且会被安装。）
2. **参数不可热改**：没有 `add_on_set_parameters_callback`，`ros2 param set` 不会改变串口行为，必须重启节点。
3. **`/imu/rpy` 的单位是度**：`rm_tracker` 按度使用；把 `publish_rpy_in_degree` 改成 false 会让追踪解算完全错误。
4. **话题名与 `rm_serial` 冲突**：`rm_serial` 在 `pub_rpy: true` 时也发 `/imu/rpy`（原样透传下位机数据，单位取决于下位机）。整车只用 `dm_imu` 供姿态，`rm_serial` 的 `pub_rpy` 保持 false。
5. **发布频率是 100 Hz 定时器，不是"每帧一次"**：IMU 上报率高于 100 Hz 时会被抽稀，低于 100 Hz 时同一帧会被去重跳过（不会重复发同一时间戳的数据）。
6. **RID 未区分**：解析出的帧不按 RID 分流，三个 float 一律当作 roll/pitch/yaw（见协议一节）。
7. **串口打不开时节点直接退出**：`/dev/ttyPortIMU` 不存在（udev 未生效、设备没插、被占用）会 `get_logger().fatal()` 并 `raise`，不会自动重连——注意与 `rm_serial`（有重连逻辑）不同。
8. **静止时无数据不会报错**：串口打开但一直没有帧时，只有 `verbose: true` 才会打印 `No frames yet from serial`；默认情况下表现为"话题没有数据、也没有任何日志"。
