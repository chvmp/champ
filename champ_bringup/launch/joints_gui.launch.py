
import os

import launch_ros
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_name = LaunchConfiguration("description_name")

    pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_config").find("champ_config")
    config = os.path.join(pkg_share, "config/joints/joints.yaml")

    declare_description_name = DeclareLaunchArgument("description_name", default_value="champ", description="Robot description")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock if true")


    # declare_description_path = DeclareLaunchArgument(name="description_path", default_value=default_model_path, description="Absolute path to robot urdf file")

    joints_calibrator_relay_node = Node(
        package="champ_bringup",
        executable="joint_calibrator_relay.py",
        output="screen",
        parameters=[
            # {'use_sim_time': use_sim_time},
            config
            ]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        
        parameters=[
            # {'use_sim_time': use_sim_time}
            ],
        remappings=[("joint_states", "joints_calibrator")],
    )
    return LaunchDescription(
        [
            # declare_use_sim_time,
            joint_state_publisher_gui_node,
            joints_calibrator_relay_node
        ]
    )
