#include "ros/ros.h"
#include <urdf_loader.h>
#include <champ_msgs/Joints.h>

#include <quadruped_base/quadruped_components.h>
#include <body_controller/body_controller.h>
#include <leg_controller/leg_controller.h>
#include <kinematics/kinematics.h>
#include <odometry/odometry.h>

ros::Publisher joints_publisher;
void publishJoints(float joints[12]);

int main(int argc, char** argv){
    ros::init(argc, argv, "my_parser");
    ros::NodeHandle nh;

    joints_publisher = nh.advertise<champ_msgs::Joints>("/champ/joint_states/raw", 10);
    champ::GaitConfig gait_config;

    gait_config.knee_orientation = "><";
    gait_config.pantograph_leg = false;
    gait_config.max_linear_velocity_x = 0.5;
    gait_config.max_linear_velocity_y = 0.25;
    gait_config.max_angular_velocity_z = 1.0;
    gait_config.swing_height = 0.1;
    gait_config.stance_depth = 0.0;
    gait_config.stance_duration = 0.25;
    gait_config.nominal_height = 0.40;

    champ::QuadrupedBase base(gait_config);
    champ::URDF::loadFromServer(base, nh);

    for(int i = 0; i < 4; i++)
    {
         for(int j = 0; j < 4; j++)
        {
            float x = base.legs[i]->joint_chain[j]->x();
            float y = base.legs[i]->joint_chain[j]->y();
            float z = base.legs[i]->joint_chain[j]->z();
            std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
        }
    }

    champ::Pose req_pose;
    req_pose.position.z = 0.40;

    champ::Velocities req_vel;
    req_vel.linear.x = gait_config.nominal_height;

    ros::Rate r(100);
    while (ros::ok())
    {
        geometry::Transformation target_foot_positions[4];
        float target_joint_positions[12];
        
        body_controller.poseCommand(target_foot_positions, req_pose);
        leg_controller.velocityCommand(target_foot_positions, req_vel);
        kinematics.inverse(target_joint_positions, target_foot_positions);
        
        publishJoints(target_joint_positions);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

void publishJoints(float joints[12])
{
    champ_msgs::Joints joints_msg;
    
    for(int i=0; i<12; i++)
    {
        joints_msg.position.push_back(joints[i]);
    }

    joints_publisher.publish(joints_msg);
}
