from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_tf_broadcaster',
            executable='enemy_tf_node',
            name='enemy_tf_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
