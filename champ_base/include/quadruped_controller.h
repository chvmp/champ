#ifndef QUADRUPED_CONTROLLER_H
#define QUADRUPED_CONTROLLER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_components.h>
#include <body_controller/body_controller.h>
#include <leg_controller/leg_controller.h>
#include <kinematics/kinematics.h>
#include <odometry/odometry.h>
#include <actuator.h>


#include <geometry_msgs/Twist.h>


class QuadrupedController
{
    ros::NodeHandle nh_;

    ros::Subscriber cmd_vel_subscriber_;
    ros::Subscriber cmd_vel_sub_;
    
    ros::Publisher joints_publisher_;   
    ros::Publisher odom_publisher_;
    ros::Publisher foot_publisher_;

    champ::GaitConfig gait_config_;
        
    champ::QuadrupedBase base_;
    champ::BodyController body_controller_;
    champ::LegController leg_controller_;
    champ::Kinematics kinematics_;
    champ::Odometry odometry_;
    champ::Actuator actuators_;
    champ::Velocities req_vel_;

    float x_pos_;
    float y_pos_;
    float heading_;
    ros::Time last_vel_time_;

    void publishJoints_(float joints[12]);
    void publishVelocities_(champ::Velocities vel);
    void publishFootPositions_(geometry::Transformation foot_positions[4]);
    void cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg);
    
    public:
        QuadrupedController();
};

#endif