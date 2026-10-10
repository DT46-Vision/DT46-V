# mindvision_camera

迈德威视（MindVision）相机驱动节点：通过 MVSDK 采集图像，按 `image_transport` 的 camera publisher 规范同步发布 `image_raw` 与 `camera_info`，并把曝光、模拟增益、RGB 增益、白平衡、饱和度、Gamma、图像翻转等相机设置暴露为 ROS 2 参数，可在运行时用 `ros2 param set` 修改。

- 上游来源：本包是 [chenjunnn/ros2_mindvision_camera](https://github.com/chenjunnn/ros2_mindvision_camera)（作者 Chen Jun，MIT License）的副本；`package.xml` 的 `<url>`、`<author>`、`<maintainer>` 与 `LICENSE` 仍是上游内容。
- 目录名与包名不同：目录是 `src/rm_vision_ros2_mindvision_camera/`，ROS 包名（`package.xml` 的 `<name>`）是 `mindvision_camera`，colcon / ros2 命令里必须用包名。
- 下面内容按本仓库源码核对：`src/mv_camera_node.cpp`、`launch/mv_launch.py`、`config/camera_params.yaml`、`config/camera_info.yaml`、`CMakeLists.txt`、`package.xml`。与上游 README 冲突之处以源码为准。

## 节点

| ROS 图中的节点名 | 可执行文件（colcon 安装后） | 作用 |
| --- | --- | --- |
| `mv_camera` | `mindvision_camera_node` | 唯一节点：初始化并打开 MindVision 相机，循环抓图，发布图像与相机内参，并把相机参数映射为 ROS 参数 |

- 节点名写死在代码里：`Node("mv_camera", options)`（`src/mv_camera_node.cpp:26`），既不是包名也不是可执行名。
- 可执行文件由 CMakeLists 的 `rclcpp_components_register_node(... EXECUTABLE ${PROJECT_NAME}_node)` 生成，即 `mindvision_camera_node`，安装位置 `install/mindvision_camera/lib/mindvision_camera/mindvision_camera_node`。
- 同一个库还注册为组件插件 `mindvision_camera::MVCameraNode`（`RCLCPP_COMPONENTS_REGISTER_NODE`），可以用 `ros2 component load` 装进其它容器进程。
- 一个进程里只有这一个节点。

## 话题接口

| 方向 | 话题名（节点在根命名空间时） | 消息类型 | QoS | 作用 |
| --- | --- | --- | --- | --- |
| 发布 | `/image_raw` | `sensor_msgs/msg/Image` | `use_sensor_data_qos=false`（默认）：KEEP_LAST，depth 10，RELIABLE，VOLATILE（`rmw_qos_profile_default`）；`=true`：KEEP_LAST，depth 5，BEST_EFFORT，VOLATILE（`rmw_qos_profile_sensor_data`） | rgb8 原始图像，每帧由 `CameraGetImageBuffer` + `CameraImageProcess` 得到 |
| 发布 | `/camera_info` | `sensor_msgs/msg/CameraInfo` | 与 `/image_raw` 完全相同的 profile（一次 `create_camera_publisher` 同时创建两个 publisher） | 相机内参，与对应图像用同一个时间戳发布 |
| 订阅 | 无 | — | — | 节点不订阅任何话题 |

说明：

- 话题名在代码里是相对名 `image_raw`（`src/mv_camera_node.cpp:84`），`image_transport` 另外注册同一命名空间下的 `camera_info`。解析用的是**节点命名空间**，不含节点名：节点没有 namespace 时就是 `/image_raw` 和 `/camera_info`（已用 `ros2 topic list` / `ros2 node list` 实测确认）。放进命名空间后是 `<ns>/image_raw`、`<ns>/camera_info`，不会自动变成 `/mv_camera/image_raw`。
- QoS 由参数 `use_sensor_data_qos` 决定，默认 `false`，即两个 publisher 都是 RELIABLE；只有显式置为 `true` 才切换成 SensorDataQoS 的 BEST_EFFORT。depth 取自 `rmw/qos_profiles.h` 中两个 profile 的定义（`ros2 topic info -v` 只显示 Reliability/Durability，不显示 depth）。
- `package.xml` 依赖 `image_transport_plugins`，所以图中还会出现 `/image_raw/compressed`、`/image_raw/compressedDepth`、`/image_raw/theora`（实测 `ros2 topic list` 可见）。
- 图像字段固定：`encoding = "rgb8"`、`step = width * 3`、`header.frame_id = "camera_optical_frame"`（写死，无参数）；相机输出格式由 `CameraSetIspOutFormat(h, CAMERA_MEDIA_TYPE_RGB8)` 固定，节点也没有分辨率 / 帧率参数（见"注意事项"）。
- 节点不发布任何 TF（`package.xml` 里没有 tf2 相关依赖）。

## 参数

"代码默认值"指 `declare_parameter` 的默认实参；"yaml 实际值"指 `config/camera_params.yaml` 里真实写入的值。

| 参数名 | 类型 | 代码默认值 | `camera_params.yaml` 实际值 | 含义 |
| --- | --- | --- | --- | --- |
| `use_sensor_data_qos` | bool | `false` | 未写（由 launch argument 注入，默认 `false`） | publisher 是否用 SensorDataQoS（BEST_EFFORT） |
| `camera_name` | string | `"mv_camera"` | `mv_camera` | 传给 `camera_info_manager::CameraInfoManager` 的相机名 |
| `camera_info_url` | string | `"package://mindvision_camera/config/camera_info.yaml"` | 同左 | 内参文件 URL，启动时 `validateURL` + `loadCameraInfo` |
| `auto_exposure` | bool | `false` | 未写 | 自动曝光开关（`CameraSetAeState`） |
| `exposure_time` | int | `5000` | `2500` | 曝光时间，单位微秒（`CameraSetExposureTime` 的单位就是微秒）。描述符附带以微秒为单位的整数范围 |
| `analog_gain` | int | 启动时从相机读到的当前值（`CameraGetAnalogGain`） | `256` | 模拟增益；范围来自相机 capability |
| `auto_white_balance` | bool | `true` | 未写 | 自动白平衡（`CameraSetWbMode`） |
| `rgb_gain.r` | int | 启动时从相机读到的当前值（`CameraGetGain`） | 未写 | 红通道数字增益 |
| `rgb_gain.g` | int | 同上 | 未写 | 绿通道数字增益 |
| `rgb_gain.b` | int | 同上 | 未写 | 蓝通道数字增益 |
| `saturation` | int | 启动时从相机读到的当前值（`CameraGetSaturation`） | 未写 | 饱和度 |
| `gamma` | int | 启动时从相机读到的当前值（`CameraGetGamma`） | 未写 | Gamma |
| `flip_image` | bool | `false` | 未写 | 是否翻转图像（`CameraFlipFrameBuffer`，按 3 通道处理） |

说明：

- yaml 里只写了 4 个参数：`camera_name`、`camera_info_url`、`exposure_time`（另有一行被注释掉的 `exposure_time: 1500`）、`analog_gain`。其余参数在 yaml 中不出现，走代码默认值，所以"代码默认值"与"yaml 实际值"真正不同的只有两处：`exposure_time`（5000 → 2500）和 `analog_gain`（相机启动时的值 → 256）。
- `exposure_time` 的类型是 **integer** 而不是 double：`double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc)`（`src/mv_camera_node.cpp:176`）里默认值字面量是整型 `5000`，模板按 `int` 推导，所以 yaml 写整数 `2500` 是匹配的（实测：声明为 integer 类型，整数 override 正常生效）。
- `analog_gain`、`rgb_gain.*`、`saturation`、`gamma` 的默认值不是常量：启动时先从相机读当前值，再以该值 declare，因此没写 yaml 时 `ros2 param get` 看到的是相机里的值。
- 所有整数参数都带 `integer_range`，范围在启动时由相机 capability 计算（例如曝光范围 = capability 的曝光行数范围 × 单行曝光时间）。rclcpp 会在设置时校验范围，超范围会被拒绝。
- 运行时修改：`ros2 param set /mv_camera exposure_time 3200`。参数回调只处理上表中的参数（`src/mv_camera_node.cpp:245-322`）；回调里 `auto_white_balance` 置为 `true` 时会先把 RGB 增益强制写成 `(100, 100, 100)`、阻塞 `50 ms`，再触发一次 `CameraSetOnceWB`。
- yaml 中如果出现代码从未 `declare` 的参数，不会报错，只是不生效（实测 rclcpp 允许存在未声明的 parameter override）。

## 使用方式

### 编译

```bash
cd /home/glennade/DT46-V
colcon build --packages-select mindvision_camera
```

CMakeLists 使用 `ament_cmake_auto`，并且只支持 `x86_64` 与 `aarch64`，其它架构直接 `message(FATAL_ERROR "Unsupport host system architecture: ...")`。编译选项含 `-Wall -Werror`。

### launch 启动

```bash
ros2 launch mindvision_camera mv_launch.py
```

launch 目录下只有 `mv_launch.py` 一个文件，它暴露 3 个 launch argument：

| launch argument | 默认值 | 作用 |
| --- | --- | --- |
| `params_file` | `<share>/mindvision_camera/config/camera_params.yaml` | 传给节点的参数文件 |
| `camera_info_url` | `package://mindvision_camera/config/camera_info.yaml` | 覆盖节点参数 `camera_info_url` |
| `use_sensor_data_qos` | `'false'`（字符串） | 覆盖节点参数 `use_sensor_data_qos`；值由 launch_ros 按 YAML 规则解析，`'false'` 会变成 bool `false` |

示例：

```bash
ros2 launch mindvision_camera mv_launch.py use_sensor_data_qos:=true
ros2 launch mindvision_camera mv_launch.py params_file:=/abs/path/my_params.yaml camera_info_url:=file:///abs/path/my_camera_info.yaml
```

### 标定

- 内参文件是 `config/camera_info.yaml`：`image_width: 1280`、`image_height: 1024`、`camera_name: mv_camera`、`distortion_model: plumb_bob`，以及 `camera_matrix` / `distortion_coefficients` / `rectification_matrix` / `projection_matrix`。节点启动时按 `camera_info_url` 读取它，之后每帧随图像发布到 `/camera_info`。
- `package.xml` 把 `camera_calibration` 和 `python3-opencv` 列为 `exec_depend`，即标定工具链需要额外安装（`ros-humble-camera-calibration`）。
- `camera_info_manager` 会创建一个 `set_camera_info` 服务，名字相对于节点命名空间，因此节点在根命名空间时是 `/set_camera_info`（实测：`/mv_camera` 节点下的服务名就是 `/set_camera_info`，既不带节点名前缀也不受 `camera_name` 参数影响）。而 ROS 2 的 `cameracalibrator` 默认查找 `camera/set_camera_info`（以及 `left_camera/`、`right_camera/`），启动时会等待该服务，找不到就打印 `Service not found`。要用标定器写回内参需要做 remap，例如 `--ros-args -r camera/set_camera_info:=/set_camera_info`，或用 `--no-service-check` 跳过检查、标定完把结果手写回 `config/camera_info.yaml`。
- 标定结果文件长什么样可以直接对照现成的 `config/camera_info.yaml` 写入；节点不校验 `image_width` / `image_height` 是否等于相机实际输出（相机分辨率没有参数可设），分辨率不一致时下游拿到的内参就是错的。
- 上游 README 提到标定结果会保存到 `/tmp/calibrationdata.tar.gz`，并给出教程链接 <https://navigation.ros.org/tutorials/docs/camera_calibration.html> 与 <http://wiki.ros.org/camera_calibration>（这是上游/ROS wiki 的说法，本仓库未核实）。

### 直接 ros2 run

```bash
ros2 run mindvision_camera mindvision_camera_node --ros-args \
  -p exposure_time:=2500 -p analog_gain:=256
```

- 可执行名 `mindvision_camera_node` 与节点名 `mv_camera` 不同：run 之后 ROS 图里是 `/mv_camera`，话题是 `/image_raw`、`/camera_info`。
- 加参数文件：`--ros-args --params-file $(ros2 pkg prefix mindvision_camera)/share/mindvision_camera/config/camera_params.yaml`（该文件的顶层键必须是 `/mv_camera`，见"注意事项"）。
- 改名：`--ros-args -r __node:=xxx`；但改名后 `camera_params.yaml` 的顶层键就对不上，参数不会生效。

## 依赖与前置条件

硬件与 SDK：

- 相机：MindVision（迈德威视）相机，代码里没有任何型号判断，只要是 MVSDK 支持的型号即可；分辨率、像素格式由相机自身配置决定，本包只固定 ISP 输出为 RGB8。
- MVSDK 动态库**没有随包提交**：`mvsdk/` 下只有 `include/CameraApi.h`、`CameraDefine.h`、`CameraStatus.h` 三个头文件，没有 lib 目录。`CMakeLists.txt` 里 `target_link_directories` 指向 `mvsdk/lib/amd64`（x86_64）或 `mvsdk/lib/arm64`（aarch64），这两个目录在当前仓库中并不存在，`-lMVSDK` 实际由系统链接路径解析（本机实测 `libmindvision_camera.so` 链接到 `/lib/libMVSDK.so`，即系统单独安装的 MindVision SDK）。因此要编译必须先在系统里装好 MindVision Linux SDK（提供 `libMVSDK.so`）。
- 不要和 `/opt/MVS` 混淆：那套 SDK 的头文件是 `MvCameraControl.h`、`MvErrorDefine.h`，库是 `libMvCameraControl*` / `libMVGigEVisionSDK*` 之类的 `Mv*` 命名，与本包要用的 MindVision `CameraApi.h` / `libMVSDK.so` 不是同一个 SDK。
- 设备权限：USB 相机通过 SDK 直接访问，不是 `/dev/video*`。需要 udev 规则给 MindVision 的 USB vendor id 放权限（`f622`，部分型号 `080b`，`MODE="666"`）。MindVision SDK 安装包自带 `88-mvusb.rules` / `99-mvusb.rules`，装完 SDK 后应位于 `/etc/udev/rules.d/`；否则非 root 用户会打开相机失败。

软件：

- ROS 2 Humble（上游 README 只声明测试过 Ubuntu 22.04 + Humble）。
- 构建依赖（`package.xml` 的 `<depend>`）：`rclcpp`、`rclcpp_components`、`sensor_msgs`、`image_transport`、`image_transport_plugins`、`camera_info_manager`；构建工具 `ament_cmake`（CMakeLists 实际用 `ament_cmake_auto`）。
- 运行依赖：`camera_calibration`、`python3-opencv`（均为 `exec_depend`）。
- 测试：`BUILD_TESTING` 打开时会启用 `ament_lint_auto`（排除了 copyright、cpplint、uncrustify，另外对 `mvsdk/include` 三个头文件做了 cpplint EXCLUDE）。

## 注意事项 / 已知问题

1. **无相机时节点会"半启动"**：`CameraEnumerateDevice` 返回 0 或 `CameraInit` 失败时，构造函数直接 `return`（`src/mv_camera_node.cpp:41-54`），而 `declareParameters()` 和 publisher 创建都在其后，所以此时节点存在于 ROS 图中但没有任何话题、也没有声明任何参数，launch/yaml 传的参数也不会生效（现象是"进程活着但 `ros2 topic list` 什么都没有"）。
2. **析构里的未初始化句柄**：成员 `int h_camera_;`（`src/mv_camera_node.cpp:327`）没有初值，析构函数却无条件调用 `CameraUnInit(h_camera_)`（`:150`）。在上面"没找到相机就 return"的路径下，`h_camera_` 从未被赋值，析构会用一个未初始化句柄调用 SDK（实际后果取决于 SDK，未实测）。
3. **三个参数被声明了但参数回调不认**：`camera_name`、`camera_info_url`、`use_sensor_data_qos` 都调用了 `declare_parameter`，但 `parametersCallback` 里没有对应分支，落到最后的 `else`，所以 `ros2 param set /mv_camera camera_info_url ...` 会返回 `successful=false`、reason 为 `Unknown parameter: ...`。这三个参数只在启动时（yaml / launch）生效。
4. **参数描述符的文字是残留值**：`param_desc` 是复用的，`rgb_gain.r/g/b` 的 description 实际是上一步留下的 `"Analog gain"`，`gamma` 的 description 是 `"Saturation"`（`:181`、`:199-209`、`:217`、`:231`）。`ros2 param describe` 会显示这些不准确的说明。
5. **曝光范围是"行数 × 单行时间"**：描述符里的 `integer_range` 用 `uiExposeTimeMin/Max * exposure_line_time` 算出（`:172-175`），是 double 乘完截断成整数，量纲是微秒；CMOS 相机只能按整行取曝光，所以设置值不一定等于实际生效值。
6. **分辨率、帧率、像素格式都不可配**：没有对应参数；帧率写死为 `CameraSetFrameSpeed(h, iFrameSpeedDesc - 1)`（`:67`，即 capability 里最后一种帧率模式），输出格式写死 RGB8（`:77`），分辨率完全由相机自身设置决定。`config/camera_info.yaml` 里固定 1280×1024，与相机实际分辨率不一致时节点不会提示，但发布的内参会与图像对不上。
7. **内参 URL 无效时静默发空内参**：`validateURL` 失败只打一条 `Invalid camera info URL` 警告（`:92-97`），之后仍然照常发布 `/camera_info`，但内容是默认构造的空内参（`image_width=0`、K 全 0），下游拿到的是无意义数据。
8. **连续取图失败会杀掉整个进程**：取图失败计数超过 5 次就 `RCLCPP_FATAL` + `rclcpp::shutdown()`（`:136-139`），进程整体退出（不是只停这个节点）。
9. **yaml 的顶层键绑死在节点全名上**：`config/camera_params.yaml` 的顶层键是 `/mv_camera`。只有节点全名正好是 `/mv_camera`（无 namespace、名字未被改）时参数才会被应用；改名或加命名空间后该文件静默失效（ROS 2 不会自动迁移，也不会报错）。
10. **包内 SDK 头文件比系统 SDK 旧**：`mvsdk/include/CameraApi.h` 与系统 `/usr/include/CameraApi.h` 内容不同（系统那份来自 MindVision Linux SDK 2.1.0.49，多出新接口，例如 `CameraSetMultiExposureGamma`）。编译使用包内头文件、链接使用系统 `libMVSDK.so`，头文件与库版本不一致的 ABI 影响**未验证**。
11. **上游 README 的失效信息**：上游 README 末尾的截图链接 `docs/rqt.png` 在本仓库不存在（没有 `docs/` 目录）；上游的 `git clone` + `rosdep` + `colcon build --symlink-install --packages-up-to mindvision_camera` 是克隆上游仓库的流程，本仓库里包已经在工作空间中，只需 `colcon build --packages-select mindvision_camera`。上游 README 建议用 rqt 的 Dynamic Reconfigure 插件动态调参，ROS 2 下对应插件由 `rqt_reconfigure` 提供（本机已安装），也可以直接用 `ros2 param set`。
12. **package.xml 的描述是模板遗留**：`<description>` 仍是 `"A template for ROS packages."`，`<version>` 为 0.1.0，`<url>` 指向上游仓库。
