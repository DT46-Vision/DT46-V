# rm_interfaces

本工作空间的自定义 ROS 2 消息包（`ament_cmake` + `rosidl_default_generators`）。所有跨节点通信的数据结构都定义在这里，**其它包必须先编译它**。

## 包内容

| 文件 | 消息 | 作用 |
| --- | --- | --- |
| `msg/ArmorInfo.msg` | `ArmorInfo` | 单块装甲板的位姿观测 |
| `msg/ArmorsMsg.msg` | `ArmorsMsg` | 一帧内所有装甲板观测（`rm_detector` → `rm_tracker`） |
| `msg/ArmorDebugInfo.msg` | `ArmorDebugInfo` | 单块装甲板的灯条像素坐标等调试信息 |
| `msg/ArmorsDebugMsg.msg` | `ArmorsDebugMsg` | 一帧的调试信息数组（`rm_detector` → 调试界面） |
| `msg/GimbalControl.msg` | `GimbalControl` | 云台控制量（`rm_tracker` → `rm_serial`） |
| `msg/Decision.msg` | `Decision` | 下位机/裁判系统决策（`rm_serial` → `rm_tracker`） |
| `msg/SerialReceive.msg` | `SerialReceive` | **当前没有任何节点使用（历史遗留）** |

## 消息定义

### ArmorInfo.msg

| 字段 | 类型 | 单位 | 含义 |
| --- | --- | --- | --- |
| `armor_id` | int32 | — | 装甲板类别 ID，取值见 `rm_detector` README 的映射表（目前只有 0/2/5/6/8/11） |
| `dx` | float32 | mm | 相机系 X（右为正） |
| `dy` | float32 | mm | 相机系 Y（下为正） |
| `dz` | float32 | mm | 相机系 Z（前为正，即距离）；PnP 失败时为 **-1** |
| `yaw` | float32 | 度 | 装甲板法向相对相机的偏航角（正对约 0） |
| `nx/ny/nz` | float32 | 无量纲 | 已有相机系单位板法线；tracker 旋转完整向量后取水平朝向，零向量回退 yaw |

### ArmorsMsg.msg

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `header` | std_msgs/Header | `stamp` 沿用相机图像时间戳，`frame_id = camera_frame` |
| `armors` | ArmorInfo[] | 本帧检测到的全部装甲板；无目标时为空数组（消息仍会发布） |

### ArmorDebugInfo.msg

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `armor_id` | int32 | 装甲板 ID |
| `color` | int32 | 灯条颜色：0 = 红，1 = 蓝 |
| `l_light_up_dx/dy`、`l_light_down_dx/dy` | float32 | 左灯条上/下端点像素坐标 |
| `r_light_up_dx/dy`、`r_light_down_dx/dy` | float32 | 右灯条上/下端点像素坐标 |

### ArmorsDebugMsg.msg

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `header` | std_msgs/Header | 同 `ArmorsMsg` |
| `armors_debug` | ArmorDebugInfo[] | 调试数组 |

### GimbalControl.msg

| 字段 | 类型 | 单位 | 含义 |
| --- | --- | --- | --- |
| `header` | std_msgs/Header | — | `frame_id = tracking_frame` |
| `can_fire` | int32 | — | 开火许可，0/1（不是 bool） |
| `yaw` | float32 | 度 | 云台目标 yaw（绝对角，非增量） |
| `pitch` | float32 | 度 | 云台目标 pitch（含弹道补偿） |

### Decision.msg

| 字段 | 类型 | 含义 | 当前是否被填充 |
| --- | --- | --- | --- |
| `header` | std_msgs/Header | `frame_id = serial_receive_frame` | 是 |
| `self_sentry_hp` | uint16 | 我方哨兵血量 | 否 |
| `self_hero_hp` | uint16 | 我方英雄血量 | 否 |
| `self_infantry_hp` | uint16 | 我方步兵血量 | 否 |
| `remain_time` | uint16 | 剩余时间（s） | 否 |
| `remain_bullet` | uint16 | 剩余弹量 | 否 |
| `match_progress` | uint8 | 比赛进程（1 = 准备，4 = 开始） | 否 |
| `occupation` | uint8 | 事件 | 否 |
| `color` | uint8 | 目标颜色：0 = 红，1 = 蓝 | 是（`rm_serial` 直通下位机 `detect_color`） |
| `yaw` | float32 | 角度 | 否 |
| `can_fire` | int32 | 开火标志 | 否 |
| `bullet_speed` | float32 | 弹速（m/s） | 是 |

> 目前 `rm_serial` 只填了 `color` 与 `bullet_speed`，其余字段是给后续接入完整裁判系统预留的（见"注意事项"）。

## 使用方式

### 编译（必须先于依赖它的包）

```bash
source /opt/ros/humble/setup.bash
cd ~/DT46-V
colcon build --packages-select rm_interfaces
source install/setup.bash
```

在 C++ 中：

```cpp
#include <rm_interfaces/msg/armors_msg.hpp>
auto pub = create_publisher<rm_interfaces::msg::ArmorsMsg>("/detector/armors_info", rclcpp::SensorDataQoS());
```

在 Python 中：

```python
from rm_interfaces.msg import Decision, GimbalControl
```

CMakeLists 里需要 `find_package(rm_interfaces REQUIRED)` + `ament_target_dependencies(<target> rm_interfaces)`，package.xml 里写 `<depend>rm_interfaces</depend>`。

### 命令行查看

```bash
ros2 interface show rm_interfaces/msg/ArmorsMsg
ros2 interface list | grep rm_interfaces
ros2 topic echo /detector/armors_info
ros2 topic echo /tracker/gimbal_control
```

## 依赖与前置条件

- 构建工具：`ament_cmake`、`rosidl_default_generators`（`build_depend`）、`rosidl_default_runtime`（`exec_depend`）。
- 消息依赖：`std_msgs`（`Header`）。
- 已声明 `<member_of_group>rosidl_interface_packages</member_of_group>`。

## 注意事项 / 已知问题

1. **单位不统一是历史设计**：`ArmorInfo.dx/dy/dz` 是 **mm**，而 `rm_tracker` 内部立刻除以 1000 转成米，追踪输出 `GimbalControl.yaw/pitch` 又是 **度**。改消息时务必一起改两侧。
2. **`can_fire` 是 `int32` 而不是 `bool`**：`GimbalControl` 与 `Decision` 都是如此，跨语言（C++/Python/MCU）传输时不要按 bool 处理。
3. **`SerialReceive.msg` 是死代码**：`CMakeLists.txt` 仍然生成它，但整个工作空间没有任何节点使用。按"不改无关代码"的约定这里只做记录，需要时可删除。
4. **`Decision` 大部分字段未被填充**：只有 `color`、`bullet_speed` 有效。`rm_serial` 目前只解析下位机 20 字节包中的 `detect_color`、`roll/pitch/yaw`、`bullet_speed`，血量、剩余时间、比赛进程等字段处于"定义了但没接"的状态；不要在业务代码里假设它们有值。
5. **消息变更必须重新编译全部下游包**：`rm_detector`、`rm_tracker`、`rm_serial` 都依赖本包，改字段后只编译 `rm_interfaces` 会出现 ABI/字段不匹配的诡异现象，建议整工作空间重编。
