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

#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

#include "ros/ros.h"

#include <champ_msgs/Contacts.h>

#include <champ/odometry/odometry.h>
#include <champ/utils/urdf_loader.h>

#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>

class StateEstimation
{
    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber foot_contacts_subscriber_;
    
    ros::Publisher velocities_publisher_;
    ros::Publisher foot_publisher_;

    tf2_ros::TransformBroadcaster base_broadcaster_;

    ros::Timer odom_data_timer_;
    ros::Timer foot_position_timer_;

    champ::Velocities current_velocities_;
    geometry::Transformation current_foot_positions_[4];
    geometry::Transformation target_foot_positions_[4];

    float x_pos_;
    float y_pos_;
    float heading_;
    ros::Time last_vel_time_;

    champ::GaitConfig gait_config_;

    champ::QuadrupedBase base_;
    champ::Odometry odometry_;

    std::vector<std::string> joint_names_;
    std::string base_name_;
    std::string node_namespace_;
    std::string odom_frame_;
    std::string base_footprint_frame_;
    std::string base_link_frame_;

    void publishVelocities_(const ros::TimerEvent& event);
    void publishFootPositions_(const ros::TimerEvent& event);

    void jointStatesCallback_(const sensor_msgs::JointState::ConstPtr& msg);
    void footContactCallback_(const champ_msgs::Contacts::ConstPtr& msg);

    visualization_msgs::Marker createMarker(geometry::Transformation foot_pos, int id, std::string frame_id);

    public:
        StateEstimation(ros::NodeHandle *nh, ros::NodeHandle *pnh);
};

#endif