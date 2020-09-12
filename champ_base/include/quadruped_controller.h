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

#ifndef QUADRUPED_CONTROLLER_H
#define QUADRUPED_CONTROLLER_H

#include "ros/ros.h"

#include <champ_msgs/Joints.h>
#include <champ_msgs/Pose.h>
#include <champ_msgs/PointArray.h>
#include <champ_msgs/ContactsStamped.h>

#include <champ/utils/urdf_loader.h>
#include <champ/body_controller/body_controller.h>
#include <champ/leg_controller/leg_controller.h>
#include <champ/kinematics/kinematics.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "tf/transform_datatypes.h"

#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class QuadrupedController
{
    ros::Subscriber cmd_vel_subscriber_;
    ros::Subscriber cmd_pose_subscriber_;
    
    ros::Publisher joint_states_publisher_;   
    ros::Publisher joint_commands_publisher_;   
    ros::Publisher foot_contacts_publisher_;

    ros::Timer loop_timer_;

    champ::Velocities req_vel_;
    champ::Pose req_pose_;

    champ::GaitConfig gait_config_;

    champ::QuadrupedBase base_;
    champ::BodyController body_controller_;
    champ::LegController leg_controller_;
    champ::Kinematics kinematics_;

    std::vector<std::string> joint_names_;

    bool publish_foot_contacts_;
    bool publish_joint_states_;
    bool publish_joint_control_;
    bool in_gazebo_;

    void controlLoop_(const ros::TimerEvent& event);
    
    void publishJoints_(float target_joints[12]);
    void publishFootContacts_(bool foot_contacts[4]);

    void cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg);
    void cmdPoseCallback_(const geometry_msgs::Pose::ConstPtr& msg);

    public:
        QuadrupedController(ros::NodeHandle *nh, ros::NodeHandle *pnh);
};

#endif