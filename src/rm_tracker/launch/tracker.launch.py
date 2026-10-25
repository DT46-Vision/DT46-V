import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 动态获取安装后的功能包配置 yaml 文件路径
    default_config_path = os.path.join(
        get_package_share_directory("rm_tracker"), "config", "tracker_params.yaml"
    )

    return LaunchDescription(
        [
            # 2. 声明 Launch 参数，允许在命令行中动态覆盖 yaml 路径
            DeclareLaunchArgument(
                name="tracker_params_file",
                default_value=default_config_path,
                description="Path to the ROS 2 parameters file for the tracker node",
            ),
            # 3. 启动 C++ 重构后的高性能追踪器节点
            Node(
                package="rm_tracker",
                executable="rm_tracker_node",  # <--- 完美匹配 CMakeLists.txt 里的可执行程序名
                name="rm_tracker",  # 节点运行时的计算图名称
                output="screen",  # 实时输出 RCLCPP 日志到终端
                emulate_tty=True,  # 强行开启 TTY 颜色模拟，C++ 日志高亮必备
                parameters=[
                    LaunchConfiguration("tracker_params_file")  # 加载对应的 yaml 参数
                ],
            ),
        ]
    )
