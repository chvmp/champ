import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression


def generate_launch_description():

    robot_name = LaunchConfiguration("robot_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    headless = LaunchConfiguration("headless")
    paused = LaunchConfiguration("paused")
    lite = LaunchConfiguration("lite")
    ros_control_file = LaunchConfiguration("ros_control_file")
    gazebo_world = LaunchConfiguration("gazebo_world")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")

    pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_gazebo").find(
        "champ_gazebo"
    )
    default_model_path = os.path.join(pkg_share, "urdf/champ.urdf.xacro")

    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="champ")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="True")
    declare_gui = DeclareLaunchArgument("gui", default_value="True")
    declare_headless = DeclareLaunchArgument("headless", default_value="False")
    declare_paused = DeclareLaunchArgument("paused", default_value="False")
    declare_lite = DeclareLaunchArgument("lite", default_value="False")
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=os.path.join(pkg_share, "/config/ros_control.yaml"),
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "gazebo_world", default_value=os.path.join(pkg_share, "/worlds/outdoor.world")
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.6")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.6"
    )

    launch_dir = os.path.join(pkg_share, "launch")
    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            gazebo_world,
        ],
        cwd=[launch_dir],
        output="screen",
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([" not ", headless])),
        cmd=["gzclient"],
        cwd=[launch_dir],
        output="screen",
    )

    start_gazebo_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            robot_name,
            "-topic",
            "/robot_description",
            "-robot_namespace",
            "",
            "-x",
            world_init_x,
            "-y",
            world_init_y,
            "-z",
            world_init_z,
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            world_init_heading,
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    #  robot_description = {"robot_description": robot_description_content}

    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare(runtime_config_package),
    #         "config",
    #         controllers_file,
    #     ]
    # )

    # robot_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_states_controller"],
    # )

    # joint_group_position_controller
    return LaunchDescription(
        [
            declare_robot_name,
            declare_use_sim_time,
            declare_gui,
            declare_headless,
            declare_paused,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            start_gazebo_spawner_cmd,
            robot_controller_spawner,
        ]
    )
