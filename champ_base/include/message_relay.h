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

#include "rclcpp/rclcpp.hpp"


#include <champ_msgs/msg/joints.hpp>
#include <champ_msgs/msg/imu.hpp>
#include <champ_msgs/msg/contacts.hpp>
#include <champ_msgs/msg/contacts_stamped.hpp>

#include <champ/utils/urdf_loader.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/quaternion.hpp>

class MessageRelay: public rclcpp::Node
{

    rclcpp::Subscription<champ_msgs::msg::Imu>::SharedPtr imu_raw_subscription_;
    rclcpp::Subscription<champ_msgs::msg::Joints>::SharedPtr joints_raw_subscription_;
    rclcpp::Subscription<champ_msgs::msg::Contacts>::SharedPtr foot_contacts_subscription_;
    
    rclcpp::Publisher<champ_msgs::msg::ContactsStamped>::SharedPtr foot_contacts_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_commands_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;

    std::vector<std::string> joint_names_;
    std::string node_namespace_;
    std::string imu_frame_;

    bool in_gazebo_;
    bool has_imu_;

    sensor_msgs::msg::Imu imu_data_;

    void IMURawCallback_(const champ_msgs::msg::Imu::SharedPtr msg);
    void jointStatesRawCallback_(const champ_msgs::msg::Joints::SharedPtr msg);
    void footContactCallback_(const champ_msgs::msg::Contacts::SharedPtr msg);

    public:
        MessageRelay();
};

#endif