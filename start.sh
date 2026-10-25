#!/usr/bin/env bash
# 1. 自动定位工作空间根目录 = 本脚本所在目录
#    (不再写死机器/用户名路径，clone 到任何机器都能直接用)
#    如需覆盖: WS_PATH=/path/to/ws ./start.sh
SELF="${BASH_SOURCE[0]:-$0}"
WS_PATH="${WS_PATH:-$(dirname "$(readlink -f "$SELF")")}"

# 2. 加载系统环境
source /opt/ros/humble/setup.bash

# 3. 加载工作空间环境 (替代了 si 命令)
if [ -f "$WS_PATH/install/setup.bash" ]; then
    source "$WS_PATH/install/setup.bash"
else
    echo "错误: 未找到 $WS_PATH/install/setup.bash"
    echo "      请先在 $WS_PATH 下执行: colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install"
    exit 1
fi

# 4. 显式指定显示器，确保 rqt 能弹出
export DISPLAY=:0

# 5. 直接执行启动指令 (替代了你的 rmhik 命令)
# ros2 launch rm_vision_bringup hik.launch.py
