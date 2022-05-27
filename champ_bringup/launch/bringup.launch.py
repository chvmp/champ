
import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    base_frame = 'base_link'

    config_pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_config").find("champ_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(package="champ_description").find("champ_description")
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, 'config/gait/gait.yaml')
    links_config = os.path.join(config_pkg_share, 'config/links/links.yaml')
    default_model_path = os.path.join(descr_pkg_share, "urdf/champ.urdf.xacro")


    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation (Gazebo) clock if true")
    declare_description_path = DeclareLaunchArgument(name="description_path", default_value=default_model_path, description="Absolute path to robot urdf file")


    # declare_description_path = DeclareLaunchArgument(name="description_path", default_value=default_model_path, description="Absolute path to robot urdf file")

    description_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('champ_description'), 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'description_path': description_path}.items(),
    )

    print(links_config)
    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'gazebo': False},
            {'publish_joint_states': True},
            {'publish_joint_control': True},
            {'publish_foot_contacts': True},
            {'joint_controller_topic': "joint_group_effort_controller/joint_trajectory"},
            links_config,
            joints_config,
            gait_config
            ],
        remappings=[
            ('/cmd_vel/smooth', '/cmd_vel')
        ]
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'orientation_from_imu': True},
            links_config,
            joints_config,
            gait_config
            ]
    )

    base_to_footprint_ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='base_to_footprint_ekf',
            output='screen',
            parameters=[
                {'base_link_frame': base_frame},
                {'use_sim_time': use_sim_time},
                os.path.join(get_package_share_directory("champ_base"), 'config', 'ekf', 'base_to_footprint.yaml'),],
            remappings=[
                ('odometry/filtered', 'odom/local')
            ]
           )

    footprint_to_odom_ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='footprint_to_odom_ekf',
            output='screen',
            parameters=[
                {'base_link_frame': base_frame},
                {'use_sim_time': use_sim_time},
                os.path.join(get_package_share_directory("champ_base"), 'config', 'ekf', 'footprint_to_odom.yaml')],
            remappings=[
                ('odometry/filtered', 'odom')
            ]
           )


    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_description_path,
            description_ld,
            quadruped_controller_node,
            state_estimator_node,
            base_to_footprint_ekf,
            footprint_to_odom_ekf
        ]
    )
