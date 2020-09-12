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

#include <message_relay.h>

MessageRelay::MessageRelay(ros::NodeHandle *nh, ros::NodeHandle *pnh)
{    
    imu_data_.orientation.w = 1.0;

    foot_contacts_publisher_   = nh->advertise<champ_msgs::ContactsStamped>("foot_contacts", 1);

    joint_states_publisher_ = nh->advertise<sensor_msgs::JointState>("joint_states", 1);
    joint_commands_publisher_ = nh->advertise<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 1);

    imu_raw_subscriber_ = nh->subscribe("imu/raw", 1, &MessageRelay::IMURawCallback_, this);
    joints_raw_subscriber_ = nh->subscribe("joint_states/raw", 1, &MessageRelay::jointStatesRawCallback_, this);
    foot_contacts_subscriber_ = nh->subscribe("foot_contacts/raw", 1, &MessageRelay::footContactCallback_, this);

    pnh->getParam("gazebo",        in_gazebo_);
    pnh->getParam("has_imu",       has_imu_);

    joint_names_ = champ::URDF::getJointNames(nh);

    node_namespace_ = ros::this_node::getNamespace();
    if(node_namespace_.length() > 1)
    {
        node_namespace_.replace(0, 1, "");
        node_namespace_.push_back('/');
    }
    else
    {
        node_namespace_ = "";
    }

    if(has_imu_)
    {
        imu_publisher_ = nh->advertise<sensor_msgs::Imu>("imu/data", 1);
        mag_publisher_ = nh->advertise<sensor_msgs::MagneticField>("imu/mag", 1);
    }

    imu_frame_ = node_namespace_ + "imu_link";
}

void MessageRelay::IMURawCallback_(const champ_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu imu_data_msg;
    sensor_msgs::MagneticField imu_mag_msg;

    imu_data_msg.header.stamp = ros::Time::now();
    imu_data_msg.header.frame_id = imu_frame_;

    imu_data_msg.orientation.w = msg->orientation.w;
    imu_data_msg.orientation.x = msg->orientation.x;
    imu_data_msg.orientation.y = msg->orientation.y;
    imu_data_msg.orientation.z = msg->orientation.z;

    imu_data_msg.linear_acceleration.x = msg->linear_acceleration.x;
    imu_data_msg.linear_acceleration.y = msg->linear_acceleration.y;
    imu_data_msg.linear_acceleration.z = msg->linear_acceleration.z;

    imu_data_msg.angular_velocity.x = msg->angular_velocity.x;
    imu_data_msg.angular_velocity.y = msg->angular_velocity.y;
    imu_data_msg.angular_velocity.z = msg->angular_velocity.z;

    imu_data_msg.orientation_covariance[0] = 0.0025;
    imu_data_msg.orientation_covariance[4] = 0.0025;
    imu_data_msg.orientation_covariance[8] = 0.0025;

    imu_data_msg.angular_velocity_covariance[0] = 0.000001;
    imu_data_msg.angular_velocity_covariance[4] = 0.000001;
    imu_data_msg.angular_velocity_covariance[8] = 0.000001;

    imu_data_msg.linear_acceleration_covariance[0] = 0.0001;
    imu_data_msg.linear_acceleration_covariance[4] = 0.0001;
    imu_data_msg.linear_acceleration_covariance[8] = 0.0001;

    imu_mag_msg.header.stamp = ros::Time::now();
    imu_mag_msg.header.frame_id = imu_frame_;

    imu_mag_msg.magnetic_field.x = msg->magnetic_field.x;
    imu_mag_msg.magnetic_field.y = msg->magnetic_field.y;
    imu_mag_msg.magnetic_field.z = msg->magnetic_field.z;

    imu_mag_msg.magnetic_field_covariance[0] = 0.000001;
    imu_mag_msg.magnetic_field_covariance[4] = 0.000001;
    imu_mag_msg.magnetic_field_covariance[8] = 0.000001;

    if(has_imu_)
    {
        imu_publisher_.publish(imu_data_msg);
        mag_publisher_.publish(imu_mag_msg);
    }
}

void MessageRelay::jointStatesRawCallback_(const champ_msgs::Joints::ConstPtr& msg)
{
    if(in_gazebo_)
    {
        trajectory_msgs::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.header.stamp = ros::Time::now();
        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(12);

        point.time_from_start = ros::Duration(1.0 / 60.0);
        for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = msg->position[i];
        }

        joints_cmd_msg.points.push_back(point);
        joint_commands_publisher_.publish(joints_cmd_msg);
    }
    else
    {   
        sensor_msgs::JointState joints_msg;

        joints_msg.header.stamp = ros::Time::now();
        joints_msg.name.resize(joint_names_.size());
        joints_msg.position.resize(joint_names_.size());
        joints_msg.name = joint_names_;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {    
            joints_msg.position[i]= msg->position[i];
        }

        joint_states_publisher_.publish(joints_msg);
    }
}

void MessageRelay::footContactCallback_(const champ_msgs::Contacts::ConstPtr& msg)
{
    champ_msgs::ContactsStamped contacts_msg;
    contacts_msg.header.stamp = ros::Time::now();
    contacts_msg.contacts.resize(4);

    for(size_t i = 0; i < 4; i++)
    {
        contacts_msg.contacts[i] = msg->contacts[i];
    }

    foot_contacts_publisher_.publish(contacts_msg);
}