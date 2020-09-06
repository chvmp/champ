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

#include <champ_msgs/Joints.h>
#include <champ_msgs/Imu.h>
#include <champ_msgs/Contacts.h>
#include <champ_msgs/ContactsStamped.h>

#include <champ/utils/urdf_loader.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>

class MessageRelay
{
    ros::Subscriber imu_raw_subscriber_;
    ros::Subscriber joints_raw_subscriber_;
    ros::Subscriber foot_contacts_subscriber_;

    ros::Publisher imu_publisher_;
    ros::Publisher mag_publisher_;
    ros::Publisher joint_states_publisher_;   
    ros::Publisher joint_commands_publisher_;   
    ros::Publisher foot_contacts_publisher_;

    std::vector<std::string> joint_names_;
    std::string node_namespace_;
    std::string imu_frame_;

    bool in_gazebo_;
    bool has_imu_;

    sensor_msgs::Imu imu_data_;

    void IMURawCallback_(const champ_msgs::Imu::ConstPtr& msg);
    void jointStatesRawCallback_(const champ_msgs::Joints::ConstPtr& msg);
    void footContactCallback_(const champ_msgs::Contacts::ConstPtr& msg);

    public:
        MessageRelay(ros::NodeHandle *nh, ros::NodeHandle *pnh);
};

#endif