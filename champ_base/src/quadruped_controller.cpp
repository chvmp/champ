 
#include "ros/ros.h"
#include <quadruped_controller.h>

#include <urdf_loader.h>
#include <champ_msgs/Joints.h>
#include <champ_msgs/Pose.h>
#include <geometry/geometry.h>
#include <boost/bind.hpp>

QuadrupedController::QuadrupedController():
    body_controller_(base_),
    leg_controller_(base_),
    kinematics_(base_),
    odometry_(base_)
{
    joints_publisher_ = nh_.advertise<champ_msgs::Joints>("/champ/joint_states/raw", 10);
    cmd_vel_sub_ = nh_.subscribe( "/champ/cmd_vel", 1, &QuadrupedController::cmdVelCallback_, this);
    
    gait_config_.knee_orientation = ">>";
    gait_config_.pantograph_leg = false;
    gait_config_.max_linear_velocity_x = 0.5;
    gait_config_.max_linear_velocity_y = 0.25;
    gait_config_.max_angular_velocity_z = 1.0;
    gait_config_.swing_height = 0.04;
    gait_config_.stance_depth = 0.0;
    gait_config_.stance_duration = 0.250;
    gait_config_.nominal_height = 0.20;

    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromServer(base_, nh_);

    for(int i = 0; i < 4; i++)
    {
         for(int j = 0; j < 4; j++)
        {
            float x = base_.legs[i]->joint_chain[j]->x();
            float y = base_.legs[i]->joint_chain[j]->y();
            float z = base_.legs[i]->joint_chain[j]->z();
            std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
        }
    }

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
        
        publishJoints_(target_joint_positions);
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