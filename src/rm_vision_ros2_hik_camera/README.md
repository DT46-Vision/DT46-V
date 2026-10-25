# hik_camera

海康（HIKVISION）USB3 工业相机的 ROS 2 驱动（ROS 2 Humble，C++），通过 MVS SDK 取流，转成 RGB8 后以 image_transport 的 CameraPublisher 发布 `sensor_msgs/Image` 与 `sensor_msgs/CameraInfo`。

上游来源：**无法从仓库内文件核实**（目录名 `rm_vision_ros2_hik_camera`、包名 `hik_camera`，包内无 git 历史/submodule/上游链接）。本目录原有的 README 内容实际是从 [chenjunnn/ros2_mindvision_camera](https://github.com/chenjunnn/ros2_mindvision_camera) 复制来的（标题写成 `ros2_mindvision_camera`、启动命令写成 `mv_launch.py`，与本包源码不符），已按源码重写。

## 节点

| 节点名（ROS 图中名字） | 可执行文件（colcon 安装后的名字） | 作用 |
| --- | --- | --- |
| `/hik_camera`（C++ 里 `Node("hik_camera", options)` 硬编码；launch 未设 `name`） | `hik_camera_node`，安装在 `install/hik_camera/lib/hik_camera/hik_camera_node` | 枚举并打开 USB 相机，取流后转 RGB8，发布 image / camera_info；提供 `/set_camera_info` 服务 |
| 同上（组件方式，进程内加载） | 组件库 `libhik_camera.so`，插件类名 `hik_camera::HikCameraNode` | 由 composable node container 加载，节点名仍是 `hik_camera` |

节点名与可执行名不同（可执行名由 `rclcpp_components_register_node(... EXECUTABLE ${PROJECT_NAME}_node)` 生成），`ros2 run` 必须写 `hik_camera_node`。

## 话题接口

节点**不订阅**任何话题；只发布下面两个话题（以及由 `camera_info_manager` 提供的标定服务）。

| 方向 | 话题名 | 消息类型 | QoS | 作用 |
| --- | --- | --- | --- | --- |
| 发布 | `/image_raw` | `sensor_msgs/msg/Image` | 由参数 `use_sensor_data_qos` 决定：true → `rmw_qos_profile_sensor_data`（Reliability=**BestEffort**、History=KeepLast、depth=**5**、Durability=Volatile）；false → `rmw_qos_profile_default`（Reliable、KeepLast、depth=**10**、Volatile） | 相机图像，`encoding = "rgb8"`，`frame_id = "camera_optical_frame"`（硬编码） |
| 发布 | `/camera_info` | `sensor_msgs/msg/CameraInfo` | 与 `/image_raw` 完全相同（同一个 CameraPublisher、同一个 qos） | 相机内参，内容由 `camera_info_url` 指定的标定文件加载；每帧的 `header` 被赋值为图像的 header（同 stamp、同 frame_id） |
| 服务端（非话题） | `/set_camera_info` | `sensor_msgs/srv/SetCameraInfo` | 服务默认 QoS（Reliable） | 由 `camera_info_manager::CameraInfoManager` 创建，`camera_calibration` 用它写回标定结果 |

话题名说明：

- 代码里写的是相对名 `"image_raw"`，节点 namespace 为 `/`（launch 没有传 namespace），所以解析为 `/image_raw`。
- `camera_info` 是**图像话题的同级兄弟**，不是子话题：`image_transport::getCameraInfoTopic()` 把话题最后一段替换为 `camera_info`，因此是 `/camera_info` 而不是 `/image_raw/camera_info`（实测：`/image_raw` → `/camera_info`；namespace 改成 `/hero` 时为 `/hero/image_raw` + `/hero/camera_info`）。
- 由于 `image_transport` 插件已安装（package.xml 依赖 `image_transport_plugins`），创建 publisher 后同一个节点下还会出现 `/image_raw/compressed`、`/image_raw/compressedDepth`、`/image_raw/theora` 这些 transport 话题；它们由插件发布，不是本节点直接写的代码。

## 参数

| 参数名 | 类型 | 代码里 `declare_parameter` 的默认值 | `config/*.yaml` 里实际写入的值 | 含义 |
| --- | --- | --- | --- | --- |
| `use_sensor_data_qos` | bool | `true` | 三个 params 文件都没写；launch 里有同名 launch argument，默认 `'false'` 且**总会**传给节点 | true 用 `rmw_qos_profile_sensor_data`，false 用 `rmw_qos_profile_default`（见上表 QoS） |
| `exposure_time` | integer | `5000` | `2000`（`camera_params.yaml`、`hero_camera_params.yaml`、`sentinel_camera_params.yaml` 都是 2000） | 曝光时间，单位微秒，写入相机的 `ExposureTime` |
| `gain` | double | 相机启动时的当前值 `f_value.fCurValue`（**不是常量**，从设备读出） | `16.0`（三个 params 文件都是 16.0） | 增益，写入相机的 `Gain` |
| `camera_name` | string | `"hik_camera"` | `hik_camera` / `hero_hik_camera` / `sentinel_hik_camera`（分别对应 camera / hero / sentinel 的 params 文件） | 传给 `CameraInfoManager` 的相机名，与标定文件里的 `camera_name` 字段对应 |
| `camera_info_url` | string | `"package://hik_camera/config/camera_info.yaml"` | 三个 params 文件都没写；launch argument 默认值与代码默认值相同 | 标定文件 URL，启动时用 `validateURL()` 校验后加载 |

补充：

- `exposure_time` 与 `gain` 带 `ParameterDescriptor`，其 `integer_range` 的上下限是**上电后从相机读到的** `ExposureTime` / `Gain` 浮点最值（会截断成整数），因此范围随设备不同，不是固定值。
- 只有 `exposure_time` 和 `gain` 能在运行时被修改，见「注意事项」。

## 使用方式

### 编译

在工作空间根目录（`/home/glennade/DT46-V`）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hik_camera --symlink-install
source install/setup.bash
```

### launch 启动

```bash
ros2 launch hik_camera hik_camera.launch.py
```

launch 文件 `launch/hik_camera.launch.py` 暴露的参数：

| launch argument | 默认值 | 作用 |
| --- | --- | --- |
| `params_file` | `<install>/share/hik_camera/config/camera_params.yaml` | 传给节点的参数文件（`camera_name`、`exposure_time`、`gain`） |
| `camera_info_url` | `package://hik_camera/config/camera_info.yaml` | 覆盖节点的 `camera_info_url` 参数 |
| `use_sensor_data_qos` | `'false'` | 覆盖节点的 `use_sensor_data_qos` 参数（launch 总是传这个参数） |

例：用 sentinel 的参数与内参文件启动（这两个文件本来就是为 bringup 准备的）：

```bash
ros2 launch hik_camera hik_camera.launch.py \
  params_file:=$(ros2 pkg prefix hik_camera)/share/hik_camera/config/sentinel_camera_params.yaml \
  camera_info_url:=package://hik_camera/config/sentinel_camera_info.yaml
```

本仓库整机启动包（目录 `src/rm_DT46`，包名 `rm_vision_bringup`）里的 `sentry.launch.py`、`hero.launch.py` 就是用 `IncludeLaunchDescription` 调本包的 `hik_camera.launch.py`，并把 `params_file` / `camera_info_url` 指到 `sentinel_*` / `hero_*` 两个文件。

### 直接 ros2 run

```bash
ros2 run hik_camera hik_camera_node --ros-args \
  -p exposure_time:=2000 -p gain:=16.0 -p camera_name:=hik_camera \
  -p camera_info_url:=package://hik_camera/config/camera_info.yaml
```

注意可执行名 `hik_camera_node` 与节点名 `hik_camera` 不同；而且直接 `ros2 run` **不会**自动加载 `config/camera_params.yaml`（那是 launch 做的），此时 `use_sensor_data_qos` 取代码默认值 `true`（BestEffort）。

### 以 composable node 方式加载

节点由 `rclcpp_components_register_node()` 注册（package `hik_camera`，plugin `hik_camera::HikCameraNode`），`ros2 component types` 已能列出（实测）：

```bash
ros2 run rclcpp_components component_container_mt          # 另开终端
ros2 component load /ComponentManager hik_camera hik_camera::HikCameraNode \
  -p exposure_time:=2000 -p gain:=16.0
```

也可以在自己的 launch 里用 `ComposableNodeContainer` + `ComposableNode(package='hik_camera', plugin='hik_camera::HikCameraNode', parameters=[...])` 加载（本包只提供 `hik_camera.launch.py`，没有提供 container 的 launch 文件）。

注意：没有相机时组件加载不会成功——节点的构造函数卡在设备枚举循环里（见「注意事项」第 5 条），`ros2 component load` 收不到响应。

### 标定

节点通过 `camera_info_manager` 提供 `/set_camera_info`，`camera_calibration` 已在 `package.xml` 里作为 `exec_depend`：

```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --ros-args -r image:=/image_raw -r camera:=/
```

- `cameracalibrator` 订阅相对话题 `image`，并创建 `camera/set_camera_info` 服务客户端；本节点的话题是 `/image_raw`、服务是 `/set_camera_info`，所以用 `-r image:=/image_raw -r camera:=/` 对齐即可（服务名对齐后能通过它启动时的 service check；也可以用 `--no-service-check` 跳过检查）。
- 标定完成后 SAVE 会调用 `/set_camera_info`，由 `camera_info_manager` 写回 `camera_info_url`。默认的 `package://hik_camera/config/camera_info.yaml` 指向包内文件，标定时建议改用 `file://` 路径保存（此建议未实测）。
- 包内已有标定文件：`config/camera_info.yaml`（`camera_name: hik_camera`）、`config/hero_camera_info.yaml`（`hero_hik_camera`）、`config/sentinel_camera_info.yaml`（`sentinel_hik_camera`），三者的 `image_width/height` 都是 1440×1080。文件里的 `camera_name` 要与 `camera_name` 参数一致。

## 依赖与前置条件

**硬件**

- 海康 USB3 工业相机（USB3 Vision）。代码只调用 `MV_CC_EnumDevices(MV_USB_DEVICE, ...)`，因此 **GigE 相机不会被枚举**；不读型号，任何 MVS SDK 支持的 USB 相机都能用。
- 多台相机同时连接时无法选择：代码固定使用 `device_list.pDeviceInfo[0]`，没有 serial number / 设备索引参数。

**SDK（已随包提交，无需额外安装 MVS）**

- `hikSDK/include/`：`MvCameraControl.h`、`PixelType.h`、`CameraParams.h`、`MvErrorDefine.h`、`MvISPErrorDefine.h`。
- `hikSDK/lib/amd64/`（`libMvCameraControl.so.4.6.1.3`）与 `hikSDK/lib/arm64/`（`libMvCameraControl.so.4.5.0.3`）；CMake 按 `CMAKE_SYSTEM_PROCESSOR` 选目录，链接 `FormatConversion`、`MediaProcess`、`MvCameraControl`、`MVRender`、`MvUsb3vTL`，并把整个 `hikSDK/lib/<arch>/` 安装到 `install/hik_camera/lib`。其它架构直接 `FATAL_ERROR`。
- 这些文件都在 git 里跟踪（`hikSDK` 不是 submodule），不需要装 HIK MVS。
- 本机另外装了系统级 MVS（`/opt/MVS/lib/64`，`libMvCameraControl.so.4.8.0.3`），版本与包内不同；实测运行安装后的 `libhik_camera.so` 时加载的是包内 SDK，但 `ldd` 会解析到 `/opt/MVS/lib/64`（机制未完全确认，出现异常时先确认实际加载的 SDK）。

**设备节点 / 权限**

- 包内没有任何 udev 规则、权限脚本或设备节点配置（`hikSDK` 只有头文件和动态库）。若枚举不到相机，需要按 HIK MVS 官方方式配置 USB 访问权限（此点无法从包内源码核实，不确定）。

**软件**

- ROS 2 Humble（`/opt/ros/humble`）、`ament_cmake`（CMake 用 `ament_cmake_auto`）。
- `package.xml` 依赖：`rclcpp`、`rclcpp_components`、`sensor_msgs`、`image_transport`、`image_transport_plugins`、`camera_info_manager`；标定另需 `camera_calibration`（`exec_depend`）。

## 注意事项 / 已知问题

1. **`use_sensor_data_qos` 的默认值，代码与 launch 不一致**：代码是 `declare_parameter("use_sensor_data_qos", true)`，而 launch 里 `DeclareLaunchArgument(..., default_value='false')` 且**无条件**把该参数传给节点。launch_ros 会把字符串按 YAML 解析（`false` → bool false），所以用 launch 启动是 Reliable（depth 10），直接 `ros2 run` 是 BestEffort（depth 5）。
2. **目录名 ≠ 包名**：目录 `rm_vision_ros2_hik_camera`，包名 `hik_camera`。`colcon --packages-select`、`ros2 launch/run` 都用 `hik_camera`。
3. **`hero_*` / `sentinel_*` 两个 config 默认 launch 不会用到**：`hik_camera.launch.py` 的默认值是 `camera_params.yaml` + `camera_info.yaml`，要用得手动传 `params_file` / `camera_info_url`（bringup 的 sentry/hero launch 就是这么做的）。
4. **params yaml 的顶层 key 是绝对节点名 `/hik_camera`**：如果 remap 节点名或加 namespace，这三个 yaml 里的参数**不会生效**（节点名在 C++ 里硬编码为 `hik_camera`）。
5. **没有相机时节点不会退出**：构造函数里先枚举设备，`nDeviceNum == 0` 时每秒打印一次 `No camera found!` 并重试，构造函数不返回，也没有超时或“无相机也能起”的模式；因此连 publisher 都不会创建，话题列表里看不到 `/image_raw`。实测以组件方式加载时 `ros2 component load` 一直收不到响应（等容器被 SIGTERM 结束后才返回 `Component constructor threw an exception: no plugins found!...`，并伴随 `rcl node's context is invalid` 报错）。运行中连续取流失败超过 5 次会打印 `Camera failed!` 并 `rclcpp::shutdown()`。
6. **运行时只能改 `exposure_time` 和 `gain`**：`parametersCallback` 只处理这两个名字，其它参数一律返回 `successful = false`、`reason = "Unknown parameter: <name>"`，所以 `ros2 param set /hik_camera camera_name ...`、`camera_info_url` 等会失败（只能在启动时通过参数文件/命令行传入）。
7. **`exposure_time` 是整型参数，`gain` 的默认值不是常量**：`declare_parameter("exposure_time", 5000)` 用的是 int 字面量（回调里也用 `param.as_int()`），而 `gain` 的默认值取自启动时相机里的当前值 `fCurValue`。两者的 `integer_range` 由相机返回的浮点最值截断得到，范围随设备变化。
8. **`camera_info_url` 校验失败只打一条 WARN**：`validateURL()` 返回 false 时只打印 `Invalid camera info URL: ...`，然后继续发布**默认构造的** `CameraInfo`（K、宽高全为 0，frame_id 才是 `camera_optical_frame`），不会让节点退出。
9. **分辨率 / 帧率 / 像素格式 / 曝光模式都没有参数**：分辨率取相机当前值（只用来开缓冲区），输出统一转成 `PixelType_Gvsp_RGB8_Packed`，`frame_id` 硬编码 `camera_optical_frame`。包内标定文件写的是 1440×1080，若相机实际输出分辨率不是这个值，内参不适用。
10. **`camera_info` 是同级话题** `/camera_info`（不是 `/image_raw/camera_info`），改 namespace 时 `/image_raw` 与 `/camera_info` 会一起变；订阅或录制时别只记一半。
11. **编译期强制 `-Wall -Werror`**（`add_definitions(-Wall -Werror)`）：任何警告都会让编译失败；只支持 `x86_64` / `aarch64`，其它架构 CMake 直接报 `Unsupport host system architecture`。
12. **安装产物的 RUNPATH 指向源码树 `hikSDK`**：`target_link_directories` 用的是源码目录，所以 `install/hik_camera/lib/libhik_camera.so` 的 RUNPATH 含 `.../rm_vision_ros2_hik_camera/hikSDK/lib/amd64`（安装目录里也拷了一份 SDK，并通过 package hook 加入 `LD_LIBRARY_PATH`）。移动或删除源码目录后运行可能找不到 SDK 库。
13. **旧 README 与本包不符（已重写）**：原标题是 `ros2_mindvision_camera`、构建命令写 `mindvision_camera`、启动命令写 `mv_launch.py`（本包只有 `launch/hik_camera.launch.py`），还提到 ROS 1 的 rqt `Dynamic Reconfigure` 和 `docs/rqt.png`（该目录不存在）；本节点用的是 ROS 2 参数回调。
14. `launch/hik_camera.launch.py` 里 `from ament_index_python.packages import get_package_prefix` 导入了但没有使用。
15. `package.xml` 的 `<description>` 仍是 `TODO: Package description`、`<license>` 仍是 `TODO: License declaration`（未填写）。
