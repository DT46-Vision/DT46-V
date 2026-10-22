# usb_camera

ROS 2 Humble 相机驱动节点：用 OpenCV 打开**本地 USB 摄像头**（V4L2 后端）或**网络 / 手机 IP 摄像头**（`web_cam` + `web_cam_url`），在独立线程里循环采帧，并发布 `image_raw` 与 `camera_info`。

- 包名 `usb_camera`，版本 0.1.0，maintainer `kielas <c1470759@outlook.com>`，License MIT（`LICENSE`，Copyright (c) 2025 kielas）。
- 上游来源：包内原有 README 只有两行（`# ROS2_USB_CAM`、`usb_camera_driver for ros2`），本包文件与本仓库现有文档中**没有**给出上游仓库 URL，无法确认具体 fork 自哪个仓库（不确定）。以下内容按当前源码 / 配置重写。

## 节点

| 节点名（ROS 图中） | 可执行文件（colcon 安装后） | 作用 |
| --- | --- | --- |
| `usb_camera_node` | `usb_camera_node`（安装到 `lib/usb_camera/usb_camera_node`） | 打开相机 → 独立线程循环采帧 → 发布 `image_raw` / `camera_info`；对 `fps`、曝光、增益支持运行时修改 |

- 节点名来自 `src/usb_camera_node.cpp` 的 `Node("usb_camera_node")`，节点没有设置 namespace，所以完整节点名是 `/usb_camera_node`。`launch/usb_camera.launch.xml` 里也显式写了 `name='usb_camera_node'`，与之一致。
- 可执行文件名与节点名相同，但两者是不同概念：`ros2 run` 用可执行名，`ros2 node list` / 参数文件顶层键用节点名。

## 话题接口

| 方向 | 话题名 | 消息类型 | QoS | 作用 |
| --- | --- | --- | --- | --- |
| 发布 | `/image_raw` | `sensor_msgs/msg/Image` | `rclcpp::SensorDataQoS()`：Reliability **BestEffort**、History KeepLast、Depth **5**、Durability Volatile | 相机图像，编码由 `encoding_str` 决定（默认 `bgr8`） |
| 发布 | `/camera_info` | `sensor_msgs/msg/CameraInfo` | 同上（BestEffort / KeepLast / depth 5 / Volatile） | 相机内参与畸变，来自 `camera_info_manager` |

补充说明：

- 源码里发布名是相对名 `"image_raw"` / `"camera_info"`，节点在默认 namespace 下，所以 ROS 图中就是 `/image_raw`、`/camera_info`（**没有**节点名前缀）。重映射示例：`--ros-args -r image_raw:=/cam/image_raw`。
- 两条消息的 `header.stamp` 都是发布瞬间的 `this->now()`（不是相机硬件时间戳），`header.frame_id` 固定为 `camera_optical_frame`。包内不发布 TF，`camera_optical_frame` 需要由使用方自己提供。
- 没有订阅任何话题，也没有自定义 service。`camera_info_manager` 会附带提供 `/usb_camera_node/set_camera_info`（`sensor_msgs/srv/SetCameraInfo`），rclcpp 默认参数服务（`set_parameters` 等）也在。
- QoS 是 BestEffort，订阅端如果用默认 Reliable 会收不到数据；订阅时请用 `SensorDataQoS` 或手动设 BestEffort。

## 参数

下表中「代码默认值」是 `src/usb_camera_node.cpp` 里 `declare_parameter` 的默认值，「yaml 实际值」是 `config/camera_params.yaml` 写入的值（由 launch 加载）。

| 参数名 | 类型 | 代码默认值 | config/camera_params.yaml 实际值 | 含义 |
| --- | --- | --- | --- | --- |
| `web_cam` | bool | `false` | `False` | `true`=打开 `web_cam_url` 网络摄像头；`false`=打开本地 USB 摄像头 |
| `web_cam_url` | string | `"http://127.0.0.1:4090/video"` | `"http://192.168.1.216:4747/video"` | 网络摄像头的流地址，仅在 `web_cam=true` 时使用（**与代码默认值不同**） |
| `camera_index` | int | `0` | `0` | USB 模式的设备号，拼成 `/dev/video{camera_index}` |
| `pixel_format` | string | `"MJPG"` | `"MJPG"` | 请求的像素格式；代码显式识别 `"YUYV"`、`"NV12"`、`"RGB3"`，其它任何值都按 MJPEG 处理 |
| `encoding_str` | string | `"bgr8"` | `"bgr8"` | 输出编码；只有等于 `"rgb8"` 时做 BGR→RGB 转换，其它值都按 `bgr8` 发布 |
| `frame_width` | int | `1920` | `640` | 采集宽度（**仅启动时读取一次，运行时改动不生效**） |
| `frame_height` | int | `1080` | `480` | 采集高度（同上，**仅启动时读取一次**） |
| `fps` | double | `120.0` | `240`（**YAML 里是整数，会导致节点启动失败，见「注意事项」第 1 条**） | 请求帧率；启动时内部变量会被相机实际返回的帧率覆盖，但 ROS 参数值不会跟着改（见「注意事项」第 6 条） |
| `exposure_auto` | bool | `true` | `True` | `true`=自动曝光（`CAP_PROP_AUTO_EXPOSURE=3`）；`false`=手动并应用 `exposure_time` |
| `exposure_time` | int | `800` | `800` | 手动曝光绝对值（`CAP_PROP_EXPOSURE`，失败回退 `V4L2_CID_EXPOSURE_ABSOLUTE`）；仅在 `exposure_auto=false` 时生效 |
| `gain` | int | `128` | `128` | 增益（`CAP_PROP_GAIN`，失败回退 `V4L2_CID_GAIN`） |
| `camera_name` | string | `"usb_camera"` | yaml 未写，用代码默认值 | 传给 `camera_info_manager` 的相机名，与 `config/camera_info.yaml` 里的 `camera_name` 一致 |
| `camera_info_url` | string | `""`（空 = 不加载标定） | yaml 未写；**launch 固定覆盖为 `package://usb_camera/config/camera_info.yaml`** | 标定文件 URL，启动时 `loadCameraInfo()` |

动态参数：

- 只有 `fps`、`exposure_auto`、`exposure_time`、`gain` 会在 `add_on_set_parameters_callback` 里处理，可用 `ros2 param set /usb_camera_node <参数> <值>` 在线修改（`exposure_time` 只在 `exposure_auto=false` 时才会被应用）。
- `pixel_format`、`frame_width`、`frame_height`、`encoding_str`、`camera_index`、`web_cam`、`web_cam_url` 运行时修改不会生效（源码注释里也写明需要重启节点）。

## 使用方式

### 编译

```bash
cd <工作空间根目录>          # 例：/home/glennade/DT46-V
colcon build --packages-select usb_camera
source install/setup.bash
```

`config` 与 `launch` 目录会被安装到 `share/usb_camera/`，launch 与 `camera_info` 都从 share 目录读取，所以**改了 `config/*.yaml` 后必须重新 `colcon build` 才生效**。

### launch 启动

```bash
ros2 launch usb_camera usb_camera.launch.xml
```

launch 暴露的可配置参数：**没有**。`launch/usb_camera.launch.xml` 里没有 `<arg>`，节点使用 `output="screen"`；写死的两项是：

| 写死的内容 | 值 | 作用 |
| --- | --- | --- |
| `params_file` | `<share>/usb_camera/config/camera_params.yaml` | 通过 `get_package_share_directory('usb_camera')` 拼出，作为 `parameters=[...]` 传入 |
| `camera_info_url` | `package://usb_camera/config/camera_info.yaml` | 作为参数覆盖传给节点 |

也就是说：切换 USB / 网络摄像头、改分辨率、改帧率都只能改 `config/camera_params.yaml`（或自己写 launch / 用 `ros2 run` 传参），不能通过 launch argument 传。

### 标定（camera_info）

- 标定文件是 `config/camera_info.yaml`，字段是 camera_info_manager / camera_calibration_parsers 的标准格式：`image_width` 640、`image_height` 480、`camera_name` `usb_camera`、`camera_matrix`、`distortion_model` `plumb_bob`、5 个 `distortion_coefficients`、`rectification_matrix`、`projection_matrix`。当前值对应 640x480（fx≈443.21、fy≈442.92、cx≈311.89、cy≈224.69）。
- 加载流程：构造函数里把 `camera_name` / `camera_info_url` 交给 `camera_info_manager::CameraInfoManager`，若 URL 非空就调用 `loadCameraInfo()`：成功打 INFO「加载相机标定文件: ...」，失败只打 WARN 且**不退出**——这种情况下每帧发布的仍是 camera_info_manager 的默认（未标定）CameraInfo，注意别把这种消息当有效内参用。
- 分辨率改了（`frame_width/height`）就要重新标定：现有 yaml 是 640x480 的，代码默认却是 1920x1080。
- 本项目使用 C++ 工具采集棋盘图片并输出标准 YAML，操作见[标定工具](../../standalone/README.md#离线回放录像与标定)。检查结果后更新本包配置、重新安装并重启节点。

### 直接 ros2 run

可执行名与节点名相同（都是 `usb_camera_node`），`ros2 run` 用的是可执行名：

```bash
# 全部使用代码默认值：/dev/video0、MJPG、1920x1080、120fps、bgr8、不加载标定
ros2 run usb_camera usb_camera_node

# 用本包的参数文件
ros2 run usb_camera usb_camera_node --ros-args \
  --params-file install/usb_camera/share/usb_camera/config/camera_params.yaml

# 用手机 / 网络 IP 摄像头（不碰 V4L2，直接打开 URL）
ros2 run usb_camera usb_camera_node --ros-args \
  -p web_cam:=true -p web_cam_url:="http://192.168.1.216:4747/video"

# 重映射输出话题
ros2 run usb_camera usb_camera_node --ros-args -r image_raw:=/cam/image_raw
```

注意：参数文件顶层键 `/usb_camera_node` 必须与节点名一致（yaml 注释里也这么写），用 `-r __node:=其他名字` 改节点名后该 yaml 不再匹配，参数会退回代码默认值。

USB 模式还是网络模式的判定只看 `web_cam`：为 `false` 时走 V4L2 底层配置 + `cap_.open(camera_index_, cv::CAP_V4L2)`，为 `true` 时跳过全部 V4L2 步骤、直接 `cap_.open(web_cam_url_)`。

## 依赖与前置条件

硬件：

- 本地 USB 摄像头（UVC / V4L2），`web_cam=false` 时按 `/dev/video{camera_index}` 访问；源码直接 `#include <linux/videodev2.h>` 并用 ioctl 配置格式与帧率，因此**只能在 Linux 上编译运行**。
- 或者网络 / IP 摄像头，`web_cam=true` 时用 `web_cam_url` 打开（yaml 里的 `http://192.168.1.216:4747/video` 是常见手机 IP 摄像头 App 的 MJPEG 地址，属于使用者自己的设备地址，换设备必须改）。
- **不需要任何厂商 SDK，也没有随包提交 SDK**：本包只有一个源文件 `src/usb_camera_node.cpp`，采集全部交给 OpenCV（USB 显式 `cv::CAP_V4L2`，网络用默认 backend）。

设备节点 / 权限：

- `/dev/video*` 必须对当前用户可读写；打不开时节点会打印 `无法打开设备文件 /dev/videoN` 并提示 `sudo chmod 777 /dev/video*`（常规做法是把用户加入 `video` 组）。网络摄像头模式不需要设备权限，也不会去开 `/dev/videoN`。
- 设备不存在时节点的行为：先报错，随后 `cv::VideoCapture` 打开失败并 FATAL「无法打开摄像头」，构造函数抛 `std::runtime_error("Camera open failed")`，进程直接终止（不是优雅退出）。

软件依赖（`package.xml` / `CMakeLists.txt`）：

- 构建：`ament_cmake`；`depend`：`rclcpp`、`sensor_msgs`、`image_transport`、`cv_bridge`、`camera_info_manager`、`OpenCV`；`exec_depend`：`launch`、`launch_ros`、`ros2launch`；C++17，编译选项 `-Wall -Wextra -Wpedantic`。
- `image_transport` 在 `package.xml` 与 `ament_target_dependencies` 里声明并链接，但源码没有使用它（发布用的是 `rclcpp::Publisher`）。
- 现有 build 目录里链接的是系统 OpenCV 4.5.4（Ubuntu 包，`libopencv_*.so.4.5.4`）。
- `camera_calibration` 只在需要重新标定时才用，不是本包依赖。

## 注意事项 / 已知问题

1. **`fps` 类型不一致，用 launch 直接启动会崩（已实测）**：`config/camera_params.yaml` 写的是 `fps: 240`（YAML 整数），而代码是 `declare_parameter<double>("fps", 120.0)`。rclcpp 不接受整数覆盖 double 参数，节点在构造阶段就抛异常退出：

   ```
   terminate called after throwing an instance of 'rclcpp::exceptions::InvalidParameterTypeException'
     what():  parameter 'fps' has invalid type: Wrong parameter type, parameter {fps} is of type {double}, setting it to {integer} is not allowed.
   ```

   实测方式：用本包源码 `colcon build` 出可执行文件后分别运行 `--params-file config/camera_params.yaml` 与 `ros2 launch usb_camera usb_camera.launch.xml`，两者都以 exit code -6 结束；把该行改成 `fps: 240.0` 后节点能正常走到相机初始化。**本 README 未修改任何配置**，使用前请自行把 yaml 里的 `fps` 改成浮点数（`240.0`）。
2. `web_cam_url` 代码默认值是 `http://127.0.0.1:4090/video`，yaml 里是 `http://192.168.1.216:4747/video`；不传参数文件时用前者，经 launch 启动时用后者。
3. 分辨率/帧率三处取值：代码默认 1920x1080 @120fps，`config/camera_params.yaml` 是 640x480 @240fps，`config/camera_info.yaml` 的内参也是 640x480（后两者一致）。
4. 参数在 yaml 里有、代码却没 `declare_parameter` 的情况：**没有**。实测把整个 `camera_params.yaml` 传给节点，除第 1 条的 `fps` 类型错误外没有「参数未声明」报错。
5. yaml 里没有写 `camera_name` 和 `camera_info_url`：`camera_name` 用代码默认 `"usb_camera"`，`camera_info_url` 由 launch 覆盖成 `package://usb_camera/config/camera_info.yaml`；因此 `ros2 run` 裸启动时**不会加载**标定文件（参数默认是空串）。
6. 帧率的「参数值」与「实际值」会不一致：启动时 `fps_` 会被 `cap_.get(cv::CAP_PROP_FPS)` 覆盖为相机实际帧率，但 ROS 参数 `fps` 仍是请求值；运行时 `ros2 param set ... fps` 若与相机实际能力差 >0.01 会打 WARN 并回退到实际值，参数本身也不会被改回实际值。要以日志（`相机确认工作参数: ... @ x.xx FPS`）为准。
7. `encoding_str` 只实现了 `"rgb8"` 一个分支：只要不是 `"rgb8"`（包括 `"bgr8"` 和其它任意字符串）都按 `bgr8` 发布，写别的值不会有任何效果也不会报错。
8. `pixel_format` 会被无条件按 4 个字符索引（`pixel_format_[0]..[3]`）拼 FOURCC，源码没有做长度检查：如果配置里写少于 4 个字符的字符串（例如空串），`std::string::operator[]` 越界属于未定义行为。此项是代码分析结论，未实测。
9. `web_cam=true` 时，曝光/增益只能走 OpenCV 的 `cap_.set()`：源码里 V4L2 回退函数 `set_v4l2_control()` 在 `web_cam_` 为真时是空操作，而且失败日志被注释掉了，因此设置失败是静默的。另外网络流通常不会理会 `frame_width/height/fps` 请求。
10. 采帧线程在 `cap_.read()` 失败时只打 WARN 并继续循环（线程退出标志只在析构函数里置位），摄像头掉线或流中断时会持续刷「捕获帧失败 (Frame Dropped or Timeout)」日志。
11. `ros2 launch usb_camera usb_camera.launch.xml` 无法通过 launch argument 改任何东西（见「launch 启动」）；改 `config` 后必须重新 `colcon build`，因为 launch 读的是 `share/usb_camera/config/`。
12. 构建产物统一在仓库根目录：`build/`、`install/`、`log/`（均被 gitignore）。历史上 `src/` 下曾误建过另一套 `src/build/`、`src/install/`、`src/log/`，其中 `src/install/usb_camera` 链接的是 2026-05-29 的旧二进制（会打印源码里已被注释掉的 `使用 USB Camera` 日志），现已删除。**只在根目录 `colcon build` 并 `source install/setup.bash`**，就不会跑到旧二进制。
13. 源码里没有任何 TF 发布，`header.frame_id` 固定为 `camera_optical_frame`，但包内没有这个坐标系，使用方需要自己在 TF 树里提供。
14. 旧 README 写的「ROS2的节点名为usb_camera_node」与源码一致；它用的标题 `ROS2_USB_CAM` 是历史名，实际包名是 `usb_camera`。
