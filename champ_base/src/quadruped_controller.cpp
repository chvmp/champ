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

QuadrupedController::QuadrupedController(const ros::NodeHandle &node_handle,
                                         const ros::NodeHandle &private_node_handle):
    nh_(node_handle),
    pnh_(private_node_handle),
    body_controller_(base_),
    leg_controller_(base_),
    kinematics_(base_),
    odometry_(base_)
{
    cmd_vel_subscriber_ = pnh_.subscribe( "/champ/cmd_vel", 1, &QuadrupedController::cmdVelCallback_, this);
    cmd_pose_subscriber_ = pnh_.subscribe( "/champ/cmd_pose", 1, &QuadrupedController::cmdPoseCallback_, this);

    joint_states_publisher_ = pnh_.advertise<sensor_msgs::JointState>("/champ/joint_states", 100);
    joint_commands_publisher_ = pnh_.advertise<trajectory_msgs::JointTrajectory>("/champ/joint_group_position_controller/command", 100);
    velocities_publisher_   = pnh_.advertise<nav_msgs::Odometry>("/odom/raw", 100);
    foot_publisher_   = pnh_.advertise<champ_msgs::PointArray>("/champ/foot/raw", 100);

    std::string knee_orientation;
    pnh_.getParam("/champ/gait/pantograph_leg",         gait_config_.pantograph_leg);
    pnh_.getParam("/champ/gait/max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    pnh_.getParam("/champ/gait/max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    pnh_.getParam("/champ/gait/max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    pnh_.getParam("/champ/gait/swing_height",           gait_config_.swing_height);
    pnh_.getParam("/champ/gait/stance_depth",           gait_config_.stance_depth);
    pnh_.getParam("/champ/gait/stance_duration",        gait_config_.stance_duration);
    pnh_.getParam("/champ/gait/nominal_height",         gait_config_.nominal_height);
    pnh_.getParam("/champ/gait/knee_orientation",       knee_orientation);
    pnh_.getParam("/champ_controller/gazebo",       in_gazebo_);
    gait_config_.knee_orientation = knee_orientation.c_str();

    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromServer(base_, pnh_);
    joint_names_ = champ::URDF::getJointNames(pnh_);

    loop_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                   &QuadrupedController::controlLoop_,
                                   this);

    odom_data_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                        &QuadrupedController::publishVelocities_, 
                                        this);

    joints_position_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                              &QuadrupedController::publishJoints_, 
                                              this);

    foot_position_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                            &QuadrupedController::publishFootPositions_, 
                                            this);

    req_pose_.position.z = gait_config_.nominal_height;

}

void QuadrupedController::controlLoop_(const ros::TimerEvent& event)
{
    geometry::Transformation target_foot_positions[4];
    float target_joint_positions[12];
    
    body_controller_.poseCommand(target_foot_positions, req_pose_);
    leg_controller_.velocityCommand(target_foot_positions, req_vel_);
    kinematics_.inverse(target_joint_positions, target_foot_positions);
    actuators_.moveJoints(target_joint_positions);

    actuators_.getJointPositions(current_joint_positions_);
    base_.updateJointPositions(current_joint_positions_);
    base_.getFootPositions(current_foot_positions_);
    odometry_.getVelocities(current_velocities_);
}

void QuadrupedController::cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

void QuadrupedController::cmdPoseCallback_(const champ_msgs::Pose::ConstPtr& msg)
{
    req_pose_.orientation.roll = msg->roll;
    req_pose_.orientation.pitch = msg->pitch;
    req_pose_.orientation.yaw = msg->yaw;

    req_pose_.position.z = msg->z * gait_config_.nominal_height;
    if(req_pose_.position.z < (gait_config_.nominal_height * 0.5))
        req_pose_.position.z = gait_config_.nominal_height * 0.5;
}

void QuadrupedController::publishJoints_(const ros::TimerEvent& event)
{
    if(in_gazebo_)
    {
        trajectory_msgs::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(12);

        point.time_from_start = ros::Duration(1.0 / 60.0);
        for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = current_joint_positions_[i];
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
            joints_msg.position[i]= current_joint_positions_[i];
        }

        joint_states_publisher_.publish(joints_msg);
    }
}

void QuadrupedController::publishVelocities_(const ros::TimerEvent& event)
{
    ros::Time current_time = ros::Time::now();

    double vel_dt = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    //rotate in the z axis
    //https://en.wikipedia.org/wiki/Rotation_matrix
    double delta_heading = current_velocities_.angular.z * vel_dt; 
    double delta_x = (current_velocities_.linear.x * cos(heading_) - current_velocities_.linear.y * sin(heading_)) * vel_dt; //m
    double delta_y = (current_velocities_.linear.x * sin(heading_) + current_velocities_.linear.y * cos(heading_)) * vel_dt; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading_ in quaternion angle
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0, 0, heading_);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    //robot's heading_ in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    odom.twist.twist.linear.x = current_velocities_.linear.x;
    odom.twist.twist.linear.y = current_velocities_.linear.y;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = current_velocities_.angular.z;

    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[35] = 0.001;

    velocities_publisher_.publish(odom);
}

void QuadrupedController::publishFootPositions_(const ros::TimerEvent& event)
{
    champ_msgs::PointArray point_msg;

    point_msg.lf.x = current_foot_positions_[0].X();
    point_msg.lf.y = current_foot_positions_[0].Y();
    point_msg.lf.z = current_foot_positions_[0].Z();

    point_msg.rf.x = current_foot_positions_[1].X();
    point_msg.rf.y = current_foot_positions_[1].Y();
    point_msg.rf.z = current_foot_positions_[1].Z();

    point_msg.lh.x = current_foot_positions_[2].X();
    point_msg.lh.y = current_foot_positions_[2].Y();
    point_msg.lh.z = current_foot_positions_[2].Z();

    point_msg.rh.x = current_foot_positions_[3].X();
    point_msg.rh.y = current_foot_positions_[3].Y();
    point_msg.rh.z = current_foot_positions_[3].Z();

    foot_publisher_.publish(point_msg);
}