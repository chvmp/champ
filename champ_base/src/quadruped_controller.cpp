 
#include "ros/ros.h"
#include <quadruped_controller.h>

#include <urdf_loader.h>
#include <champ_msgs/Joints.h>
#include <champ_msgs/Pose.h>
#include <champ_msgs/PointArray.h>

#include <geometry/geometry.h>
#include <boost/bind.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

QuadrupedController::QuadrupedController():
    body_controller_(base_),
    leg_controller_(base_),
    kinematics_(base_),
    odometry_(base_)
{
    cmd_vel_sub_ = nh_.subscribe( "/champ/cmd_vel", 1, &QuadrupedController::cmdVelCallback_, this);

    joints_publisher_ = nh_.advertise<champ_msgs::Joints>("/champ/joint_states/raw", 10);
    odom_publisher_ = nh_.advertise<nav_msgs::Odometry>("odom/raw", 50);
    foot_publisher_ = nh_.advertise<champ_msgs::PointArray>("/champ/foot/raw", 50);

    std::string knee_orientation;
    nh_.getParam("/champ/gait/pantograph_leg",         gait_config_.pantograph_leg);
    nh_.getParam("/champ/gait/max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    nh_.getParam("/champ/gait/max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    nh_.getParam("/champ/gait/max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    nh_.getParam("/champ/gait/swing_height",           gait_config_.swing_height);
    nh_.getParam("/champ/gait/stance_depth",           gait_config_.stance_depth);
    nh_.getParam("/champ/gait/stance_duration",        gait_config_.stance_duration);
    nh_.getParam("/champ/gait/nominal_height",         gait_config_.nominal_height);
    nh_.getParam("/champ/gait/knee_orientation",       knee_orientation);
    gait_config_.knee_orientation = knee_orientation.c_str();

    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromServer(base_, nh_);

    champ::Pose req_pose;
    req_pose.position.z = gait_config_.nominal_height;

    champ::Velocities req_vel;

    ros::Rate r(200);
    while (ros::ok())
    {
        geometry::Transformation target_foot_positions[4];
        float target_joint_positions[12];
        
        body_controller_.poseCommand(target_foot_positions, req_pose);
        leg_controller_.velocityCommand(target_foot_positions, req_vel_);
        kinematics_.inverse(target_joint_positions, target_foot_positions);
        actuators_.moveJoints(target_joint_positions);

        geometry::Transformation current_foot_positions[4];
        float current_joint_positions[12];
        champ::Velocities current_speed;

        actuators_.getJointPositions(current_joint_positions);
        base_.updateJointPositions(current_joint_positions);
        base_.getFootPositions(current_foot_positions);
        odometry_.getVelocities(current_speed);

        publishJoints_(current_joint_positions);
        publishVelocities_(current_speed);
        publishFootPositions_(current_foot_positions);

        ros::spinOnce();
        r.sleep();
    }
}

void QuadrupedController::publishJoints_(float joints[12])
{
    champ_msgs::Joints joints_msg;
    
    for(int i = 0; i < 12; i++)
    {
        joints_msg.position.push_back(joints[i]);
    }

    joints_publisher_.publish(joints_msg);
}

void QuadrupedController::cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

void QuadrupedController::publishVelocities_(champ::Velocities vel)
{
    ros::Time current_time = ros::Time::now();

    double vel_dt = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    double delta_heading = vel.angular.z * vel_dt; //radians
    double delta_x = (vel.linear.x * cos(heading_) - vel.linear.y * sin(heading_)) * vel_dt; //m
    double delta_y = (vel.linear.x * sin(heading_) + vel.linear.y * cos(heading_)) * vel_dt; //m

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0,0,heading_);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    //robot's position in x,y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    //robot's heading in quaternion
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    //linear speed from encoders
    odom.twist.twist.linear.x = vel.linear.x;
    odom.twist.twist.linear.y = vel.linear.y;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vel.angular.z;

    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[35] = 0.001;

    odom_publisher_.publish(odom);
}

void QuadrupedController::publishFootPositions_(geometry::Transformation foot_positions[4])
{
    champ_msgs::PointArray point_msg;

    point_msg.lf.x = foot_positions[0].X();
    point_msg.lf.y = foot_positions[0].Y();
    point_msg.lf.z = foot_positions[0].Z();

    point_msg.rf.x = foot_positions[1].X();
    point_msg.rf.y = foot_positions[1].Y();
    point_msg.rf.z = foot_positions[1].Z();

    point_msg.lh.x = foot_positions[2].X();
    point_msg.lh.y = foot_positions[2].Y();
    point_msg.lh.z = foot_positions[2].Z();

    point_msg.rh.x = foot_positions[3].X();
    point_msg.rh.y = foot_positions[3].Y();
    point_msg.rh.z = foot_positions[3].Z();

    foot_publisher_.publish(point_msg);
}