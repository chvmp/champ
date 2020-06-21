/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MESSAGE_RELAY_H
#define QUADRUPED_CONTROLLER_H

#include "ros/ros.h"

#include <urdf_loader.h>
#include <champ_msgs/Joints.h>
#include <champ_msgs/Pose.h>
#include <champ_msgs/PointArray.h>
#include <champ_msgs/Imu.h>
#include <champ_msgs/Velocities.h>
#include <geometry/geometry.h>
#include <odometry/odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

class MessageRelay
{
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    ros::Subscriber cmd_pose_subscriber_;
    ros::Subscriber foot_raw_subscriber_;
    ros::Subscriber imu_raw_subscriber_;
    ros::Subscriber joints_raw_subscriber_;
    ros::Subscriber velocities_raw_subscriber_;

    ros::Publisher foot_publisher_;
    ros::Publisher imu_publisher_;
    ros::Publisher mag_publisher_;
    ros::Publisher joint_states_publisher_;   
    ros::Publisher joint_commands_publisher_;   
    ros::Publisher odometry_publisher_;

    tf2_ros::TransformBroadcaster base_broadcaster_;

    std::vector<std::string> joint_names_;
    std::string base_name_;
    std::string node_namespace_;
    std::string odom_frame_;
    std::string base_footprint_frame_;
    std::string base_link_frame_;
    std::string imu_frame_;

    bool in_gazebo_;
    bool has_imu_;

    float x_pos_;
    float y_pos_;
    float heading_;
    ros::Time last_vel_time_;

    champ::Pose req_pose_;
    sensor_msgs::Imu imu_data_;

    champ::Velocities current_velocities_;

    visualization_msgs::Marker createMarker(geometry::Transformation foot_pos, int id, std::string frame_id);
    void footRawCallback(const champ_msgs::PointArray::ConstPtr& msg);
    void IMURawCallback(const champ_msgs::Imu::ConstPtr& msg);
    void jointStatesRawCallback(const champ_msgs::Joints::ConstPtr& msg);
    void odometryRawCallback(const champ_msgs::Velocities::ConstPtr& msg);
    void cmdPoseCallback_(const champ_msgs::Pose::ConstPtr& msg);

    public:
        MessageRelay(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
};

#endif