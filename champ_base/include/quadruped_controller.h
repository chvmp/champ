#ifndef QUADRUPED_CONTROLLER_H
#define QUADRUPED_CONTROLLER_H

#include "ros/ros.h"

#include <urdf_loader.h>
#include <champ_msgs/Joints.h>
#include <champ_msgs/Pose.h>
#include <champ_msgs/PointArray.h>

#include <geometry/geometry.h>
#include <boost/bind.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_components.h>
#include <body_controller/body_controller.h>
#include <leg_controller/leg_controller.h>
#include <kinematics/kinematics.h>
#include <odometry/odometry.h>
#include <actuator.h>

#include <geometry_msgs/Twist.h>
#include <boost/thread.hpp>

class QuadrupedController
{
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber cmd_vel_subscriber_;
    ros::Subscriber cmd_vel_sub_;
    
    ros::Publisher joints_publisher_;   
    ros::Publisher velocities_publisher_;
    ros::Publisher foot_publisher_;

    ros::Timer loop_timer_;
    ros::Timer odom_data_timer_;
    ros::Timer joints_position_timer_;
    ros::Timer foot_position_timer_;

    champ::Velocities req_vel_;
    champ::Velocities current_velocities_;
    float current_joint_positions_[12];
    geometry::Transformation current_foot_positions_[4];

    float x_pos_;
    float y_pos_;
    float heading_;
    ros::Time last_vel_time_;

    champ::GaitConfig gait_config_;

    champ::QuadrupedBase base_;
    champ::BodyController body_controller_;
    champ::LegController leg_controller_;
    champ::Kinematics kinematics_;
    champ::Odometry odometry_;
    champ::Actuator actuators_;

    void controlLoop_(const ros::TimerEvent& event);
    void publishJoints_(const ros::TimerEvent& event);
    void publishVelocities_(const ros::TimerEvent& event);
    void publishFootPositions_(const ros::TimerEvent& event);

    void cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg);
    
    public:
        QuadrupedController(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &private_node_handle);
};

#endif