import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ==================== 1. 获取各个功能包的路径 ====================
    hik_camera_pkg  = get_package_share_directory('hik_camera')
    rm_detector_pkg = get_package_share_directory('rm_detector')
    rm_tracker_pkg  = get_package_share_directory('rm_tracker')
    dm_imu_pkg      = get_package_share_directory('dm_imu')
    rm_serial_pkg   = get_package_share_directory('rm_serial')

    # ==================== 2. 明确写死 Sentry 专属的参数文件路径 ====================
    # 相机参数 (Sentry)
    sentinel_camera_params = os.path.join(hik_camera_pkg, 'config', 'sentinel_camera_params.yaml')
    sentinel_camera_info   = 'package://hik_camera/config/sentinel_camera_info.yaml'

    # 算法参数 (Sentry) 
    sentry_detector_params = os.path.join(rm_detector_pkg, 'config', 'detector_params_sentry.yaml')
    sentry_tracker_params  = os.path.join(rm_tracker_pkg, 'config', 'tracker_params_sentry.yaml')

    # 硬件底层参数 (通常全兵种通用)
    dm_imu_params = os.path.join(dm_imu_pkg, 'config', 'dm_imu_params.yaml')
    serial_params = os.path.join(rm_serial_pkg, 'config', 'rm_serial_params.yaml')

    # RQT 界面配置
    perspective_file = os.path.expanduser('~/DT46_V/Kielas_Vision.perspective')

    # ==================== 3. 组装 LaunchDescription ====================
    return LaunchDescription([

        # ----------- 1. 启动底层硬件驱动 (通用) -----------
        Node(
            package="rm_serial",
            executable="rm_serial_node",
            name="rm_serial",
            output="screen",
            emulate_tty=True,
            parameters=[serial_params],
        ),

        Node(
            package="dm_imu",
            executable="dm_imu_node",
            name="dm_imu",
            output="screen",
            emulate_tty=True,
            parameters=[dm_imu_params],
        ),

        # ----------- 2. 启动海康相机 (强绑定 Sentry 标定参数) -----------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hik_camera_pkg, 'launch', 'hik_camera.launch.py')
            ),
            launch_arguments={
                'params_file': sentinel_camera_params,
                'camera_info_url': sentinel_camera_info,
                'use_sensor_data_qos': 'false' # 根据你的需求开启或关闭
            }.items()
        ),

        # ----------- 3. 启动核心视觉算法 (强绑定 Sentry 算法参数) -----------
        Node(
            package="rm_detector",
            executable="rm_detector_node",
            name="rm_detector",
            output="screen",
            emulate_tty=True,
            parameters=[sentry_detector_params],
        ),

        Node(
            package="rm_tracker",
            executable="rm_tracker_node",
            name="rm_tracker",
            output="screen",
            emulate_tty=True,
            parameters=[sentry_tracker_params],
        ),

        # ----------- 4. 启动 RQT 调试界面 -----------
        Node(
            package="rqt_gui",
            executable="rqt_gui",
            name="rqt_gui",
            arguments=["--perspective-file", perspective_file],
            output="screen",
        ),
    ])
