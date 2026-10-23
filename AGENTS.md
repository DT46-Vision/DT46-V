# AGENTS.md

RoboMaster vision/auto-aim system: standalone C++20 applications and ROS 2 Humble workspace (Ubuntu 22.04). C++ packages (`rm_detector`, `rm_tracker`, `rm_serial`, `dm_imu`, `dt46_hardware`, `dt46_tools`), camera drivers with vendored SDKs. Native build and workflow: `standalone/README.md`. ROS/hardware validation remains separate from Windows native tests.

## 仿真接入工作的 Git 约束

必须遵守 [工作区 Git 分支隔离规范](../README.md#git-分支隔离)，优先于通用技能中的默认主分支流程。当前个人工作分支及 GlennAde 原始起点见 [原生运行说明](standalone/README.md)。

## 省额度

- 检索限定单个包。不要打开 `hikSDK/`、`mvsdk/`、`build/`、`install/`、`*.so`、`*.onnx`。
- 编译用 `colcon build --packages-select <pkg>`。用户没要求就不要全量编译或清空 `build/`。
- 包名、launch、分机器人 yaml、串口和编译细节在 skill `dt46`。需要时再读那个文件，不要为了找路径扫整个仓库。

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
