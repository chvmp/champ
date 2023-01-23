
from os import path

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    config_path = path.join(
        get_package_share_directory('champ_base'),
        'config',
        'velocity_smoother',
        'velocity_smoother.yaml'
    )

    velocity_smoother_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(
                get_package_share_directory("yocs_velocity_smoother"),
                "launch",
                "velocity_smoother.launch.py"
            )
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file", default=config_path),
            "raw_cmd_vel_topic": LaunchConfiguration("raw_cmd_vel_topic", default="cmd_vel"),
            "smooth_cmd_vel_topic": LaunchConfiguration("smooth_cmd_vel_topic", default="cmd_vel/smooth"),
            "robot_cmd_vel_topic": LaunchConfiguration("robot_cmd_vel_topic", default="cmd_vel"),
            "odom_topic": LaunchConfiguration("odom_topic", default="odom")
        }.items()
    )

    return LaunchDescription([velocity_smoother_launch])
