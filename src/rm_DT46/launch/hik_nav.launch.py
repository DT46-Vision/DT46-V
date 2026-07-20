import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---------------- 相机参数 ----------------
    params_file = os.path.join(
        get_package_share_directory("hik_camera"), "config", "camera_params.yaml"
    )
    camera_info_url = "package://hik_camera/config/camera_info.yaml"

    # ---------------- 装甲板检测参数 ----------------
    detector_params_sentry_file = os.path.join(
        get_package_share_directory("rm_detector"),
        "config",
        "detector_params_sentry.yaml",
    )

    # ---------------- 装甲板追踪参数 ----------------
    tracker_params_sentry_file = os.path.join(
        get_package_share_directory("rm_tracker"),
        "config",
        "tracker_params_sentry.yaml",
    )

    dm_imu_params_file = os.path.join(
        get_package_share_directory("dm_imu"), "config", "dm_imu_params.yaml"
    )

    # ---------------- rqt 界面配置 ----------------
    perspective_file = os.path.expanduser(
        "~/DT46-V/Kielas_Vision.perspective"
    )
