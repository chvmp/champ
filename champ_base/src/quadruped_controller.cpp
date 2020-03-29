 
#include "ros/ros.h"
#include <quadruped_controller.h>

#include <urdf_loader.h>
#include <champ_msgs/Joints.h>
#include <champ_msgs/Pose.h>
#include <geometry/geometry.h>

QuadrupedController::QuadrupedController()
{
    joints_publisher_ = nh_.advertise<champ_msgs::Joints>("/champ/joint_states/raw", 10);
    
    gait_config.knee_orientation = "><";
    gait_config.pantograph_leg = false;
    gait_config.max_linear_velocity_x = 0.5;
    gait_config.max_linear_velocity_y = 0.25;
    gait_config.max_angular_velocity_z = 1.0;
    gait_config.swing_height = 0.1;
    gait_config.stance_depth = 0.0;
    gait_config.stance_duration = 0.25;
    gait_config.nominal_height = 0.35;


    champ::QuadrupedBase base(gait_config);
    champ::URDF::loadFromServer(base, nh_);

    champ::BodyController body_controller(base);
    champ::LegController leg_controller(base);
    champ::Kinematics kinematics(base);
    champ::Odometry odometry(base);

    champ::Pose req_pose;
    req_pose.position.z = gait_config.nominal_height;

    champ::Velocities req_vel;
    req_vel.linear.x = gait_config.max_linear_velocity_x;

    ros::Rate r(100);
    while (ros::ok())
    {
        geometry::Transformation target_foot_positions[4];
        float target_joint_positions[12];
        
        body_controller.poseCommand(target_foot_positions, req_pose);
        leg_controller.velocityCommand(target_foot_positions, req_vel);
        kinematics.inverse(target_joint_positions, target_foot_positions);
        
        publishJoints_(target_joint_positions);
        ros::spinOnce();
        r.sleep();
    }

}

void QuadrupedController::publishJoints_(float joints[12])
{
    champ_msgs::Joints joints_msg;
    
    for(int i=0; i<12; i++)
    {
        joints_msg.position.push_back(joints[i]);
    }

    joints_publisher_.publish(joints_msg);
}