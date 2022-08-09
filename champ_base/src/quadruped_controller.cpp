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

#include <quadruped_controller.h>

champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time& time)
{
  return time.nanoseconds() / 1000ul;
}

QuadrupedController::QuadrupedController():
    Node("quadruped_controller_node",rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)),
    clock_(*this->get_clock()),
    body_controller_(base_),
    leg_controller_(base_, rosTimeToChampTime(clock_.now())),
    kinematics_(base_)
{
    std::string joint_control_topic = "joint_group_position_controller/command";
    std::string knee_orientation;
    std::string urdf = "";

    double loop_rate = 200.0;

    this->get_parameter("gait.pantograph_leg",         gait_config_.pantograph_leg);
    this->get_parameter("gait.max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    this->get_parameter("gait.max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    this->get_parameter("gait.com_x_translation",      gait_config_.com_x_translation);
    this->get_parameter("gait.swing_height",           gait_config_.swing_height);
    this->get_parameter("gait.stance_depth",           gait_config_.stance_depth);
    this->get_parameter("gait.stance_duration",        gait_config_.stance_duration);
    this->get_parameter("gait.nominal_height",         gait_config_.nominal_height);
    this->get_parameter("gait.knee_orientation",       knee_orientation);
    this->get_parameter("publish_foot_contacts",       publish_foot_contacts_);
    this->get_parameter("publish_joint_states",        publish_joint_states_);
    this->get_parameter("publish_joint_control",       publish_joint_control_);
    this->get_parameter("gazebo",                      in_gazebo_);
    this->get_parameter("joint_controller_topic",      joint_control_topic);
    this->get_parameter("loop_rate",                   loop_rate);
    this->get_parameter("urdf",                        urdf);
    
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel/smooth", 10, std::bind(&QuadrupedController::cmdVelCallback_, this,  std::placeholders::_1));
    cmd_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "body_pose", 1,  std::bind(&QuadrupedController::cmdPoseCallback_, this,  std::placeholders::_1));
    
    if(publish_joint_control_)
    {
        joint_commands_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_control_topic, 10);
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

    if(publish_foot_contacts_ && !in_gazebo_)
    {
        foot_contacts_publisher_   = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);
    }

    gait_config_.knee_orientation = knee_orientation.c_str();
    
    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromString(base_, this->get_node_parameters_interface(), urdf);
    joint_names_ = champ::URDF::getJointNames(this->get_node_parameters_interface());
    std::chrono::milliseconds period(static_cast<int>(1000/loop_rate));

    loop_timer_ = this->create_wall_timer(
         std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&QuadrupedController::controlLoop_, this));
    req_pose_.position.z = gait_config_.nominal_height;
}

void QuadrupedController::controlLoop_()
{
    float target_joint_positions[12];
    geometry::Transformation target_foot_positions[4];
    bool foot_contacts[4];

    body_controller_.poseCommand(target_foot_positions, req_pose_);

    leg_controller_.velocityCommand(target_foot_positions, req_vel_, rosTimeToChampTime(clock_.now()));
    kinematics_.inverse(target_joint_positions, target_foot_positions);

    publishFootContacts_(foot_contacts);
    publishJoints_(target_joint_positions);
}

void QuadrupedController::cmdVelCallback_(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

void QuadrupedController::cmdPoseCallback_(const geometry_msgs::msg::Pose::SharedPtr msg)
{   
    
    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    req_pose_.orientation.roll = roll;
    req_pose_.orientation.pitch = pitch;
    req_pose_.orientation.yaw = yaw;

    req_pose_.position.x = msg->position.x;
    req_pose_.position.y = msg->position.y;
    req_pose_.position.z = msg->position.z +  gait_config_.nominal_height;
}

void QuadrupedController::publishJoints_(float target_joints[12])
{   
    if(publish_joint_control_)
    {
        trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.header.stamp = clock_.now();
        joints_cmd_msg.header.stamp.sec = 0;
        joints_cmd_msg.header.stamp.nanosec = 0;
        
        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        point.time_from_start = rclcpp::Duration::from_seconds(1.0 / 60.0);
        for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = target_joints[i];
        }

        joints_cmd_msg.points.push_back(point);
        joint_commands_publisher_->publish(joints_cmd_msg);
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        sensor_msgs::msg::JointState joints_msg;

        joints_msg.header.stamp = clock_.now();

        joints_msg.name.resize(joint_names_.size());
        joints_msg.position.resize(joint_names_.size());
        joints_msg.name = joint_names_;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {    
            joints_msg.position[i]= target_joints[i];
        }

        joint_states_publisher_->publish(joints_msg);
    }
}

void QuadrupedController::publishFootContacts_(bool foot_contacts[4])
{
    if(publish_foot_contacts_ && !in_gazebo_)
    {
        champ_msgs::msg::ContactsStamped contacts_msg;
        contacts_msg.header.stamp = clock_.now();
        contacts_msg.contacts.resize(4);
        
        std::string s2;
       for(size_t i = 0; i < 4; i++)
        {
            //This is only published when there's no feedback on the robot
            //that a leg is in contact with the ground
            //For such cases, we use the stance phase in the gait for foot contacts
            contacts_msg.contacts[i] = base_.legs[i]->gait_phase();
            s2.append(std::to_string(contacts_msg.contacts[i]) + " ");
        }
        foot_contacts_publisher_->publish(contacts_msg);
    }
}
