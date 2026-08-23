# AGENTS.md

RoboMaster vision/auto-aim system: ROS 2 Humble workspace (Ubuntu 22.04). C++ packages (`rm_detector`, `rm_tracker`), Python packages (`rm_serial`, `dm_imu`), camera drivers with vendored SDKs.

## Build & run

- Source ROS first: `source /opt/ros/humble/setup.bash` (only needed once per shell).
- Full build from repo root (use these exact flags; clangd relies on the exported compile_commands.json, see `.vscode/settings.json`):
  ```bash
  colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install
  source install/setup.bash
  ```
- Focused build: `colcon build --packages-select <pkg>`. `rm_interfaces` (custom msgs) must be built first whenever `build/`/`install/` are missing.
- Both C++ packages compile with `-O3 -march=native` (see their CMakeLists) — builds are slow-ish.
- `build/`, `install/`, `log/` are gitignored. Their current on-disk state is stale/partial (e.g. `build/rm_tracker` still has the old Python layout). If a package behaves oddly after edits, `rm -rf build install log` and rebuild.
- No tests, no CI, no linters configured — nothing to run besides the build.

## Naming gotchas

- `src/rm_DT46/package.xml` declares `<name>rm_vision_bringup</name>`. Launch commands use `rm_vision_bringup`, never `rm_DT46`.
- Launch entrypoints (in `src/rm_DT46/launch/`): `hik.launch.py` (sentry default), `mv.launch.py` (infantry), `sentry/hero/infantry.launch.py`, `*_nav.launch.py` (vision-only, no serial/IMU), `bag_record.launch.py` (rosbag replay). Each robot type loads its own config triplet, e.g. `detector_params_sentry.yaml` + `tracker_params_sentry.yaml` + `sentinel_camera_params.yaml`.
- Current branch is `Tracker_to_C++`: `rm_tracker` has been ported from Python to C++ (core lib `tracker_core` + `rm_tracker_node` in `src/rm_tracker/{src,include}`). The root README still describes the Python tracker — treat it as stale; trust the source.
- Detector loads `src/rm_detector/model/RM_Armor_CNN_sim.onnx` via OpenCV DNN (`cv::dnn::readNetFromONNX`, CUDA if available, CPU fallback) — README's "ONNXRuntime" claim is wrong.

## Runtime prerequisites

- `rm_serial` and `dm_imu` configs hardcode udev symlinks: `/dev/ttyPortMCU` (115200) and `/dev/ttyPortIMU` (921600). Without the udev rules (see README §3) nodes fail to open ports. Camera nodes need physical cameras.
- Machine-specific hardcoded paths, don't trust them: launch files reference `~/DT46_V/Kielas_Vision.perspective` (rqt layout); `start.sh` hardcodes `WS_PATH=/home/kie-dt46/DT46_V` and `DISPLAY=:0`.

## Conventions

- `thoughts.md` (team rules): keep changes minimal, follow engineering standards, write clean reusable code.
- Commit messages are short, in Chinese, typically "修改xxx" (fix/change X) — follow that style.
- Camera SDKs (`hikSDK/`, `mvsdk/` inside the camera packages) are committed to git — no external SDK install required.

## coding rules
- 1. 如果不确定，请询问而非猜测；当存在歧义时，不要默默选择；如果有更简单的方法，就直言不讳；指出不清楚的地方并寻求澄清。
- 2. 除了被要求的内容之外，没有其他功能；一次性代码无抽象；没有“灵活性”或“可配置性”，只要是没被要求的；对于不可能的情景，没有错误处理；如果200行可以变成50行，那就重写它。
- 3. 不要“改进”相邻的代码、注释或格式；不要重构那些没坏掉的东西；即使你会用不同的方式，也要匹配现有的风格；如果你发现了无关的死代码，要提及——不要删除。
- 4. 对于多步骤任务，请提出简要计划。
- 5. 命名说人话：get_image 比 proc_img 好。bbox_to_xyz_camera 比 b2x 好。变量名不是越短越好，是越准确越好。
- 6. 一个函数只做一件事：80 行的函数一般可以拆成 3 个。拆不出来就说明你没想清楚流程。
- 7. 别写注释解释代码在干什么，写注释解释为什么这么干：# 循环 10 次 ← 废话，for i in range(10) 已经告诉你了# 取 10 帧才检测是为了等相机曝光稳定 ← 这才值得写
- 8. import 顺序和分组：标准库 → 第三方 → 本项目。中间空一行。一眼能看出依赖了哪些外部包。
- 9. 类型标注：输入输出标注类型不只是给 IDE 用，是给你自己用的。三个月后回头看 def detect(image, text, mode, scale)和 def detect(image: np.ndarray, text: str, mode: str = "phrase") -> list[dict]后者你能直接上手，前者你得再看一遍实现。
