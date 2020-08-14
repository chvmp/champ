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

#include <state_estimation.h>

StateEstimation::StateEstimation(ros::NodeHandle *nh, ros::NodeHandle *pnh):
    odometry_(base_)
{
    joint_states_subscriber_ = nh->subscribe("joint_states", 100, &StateEstimation::jointStatesCallback_, this);
    foot_contacts_subscriber_ = nh->subscribe("foot_contacts", 100, &StateEstimation::footContactCallback_, this);
    
    velocities_publisher_   = nh->advertise<nav_msgs::Odometry>("odom/raw", 100);
    foot_publisher_   = nh->advertise<visualization_msgs::MarkerArray>("foot", 100);

    nh->getParam("links_map/base", base_name_);
    nh->getParam("gait/odom_scaler", gait_config_.odom_scaler);

    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromServer(base_, nh);
    joint_names_ = champ::URDF::getJointNames(nh);

    node_namespace_ = ros::this_node::getNamespace();
    if(node_namespace_.length() > 1)
    {
        node_namespace_.replace(0, 1, "");
        node_namespace_.push_back('/');
    }
    else
    {
        node_namespace_ = "";
    }
    odom_frame_ = node_namespace_ + "odom";
    base_footprint_frame_ = node_namespace_ + "base_footprint";
    base_link_frame_ = node_namespace_ + base_name_;

    odom_data_timer_ = pnh->createTimer(ros::Duration(0.02),
                                        &StateEstimation::publishVelocities_, 
                                        this);

    foot_position_timer_ = pnh->createTimer(ros::Duration(0.02),
                                            &StateEstimation::publishFootPositions_, 
                                            this);
}

void StateEstimation::jointStatesCallback_(const sensor_msgs::JointState::ConstPtr& msg)
{
    float current_joint_positions[12];

    for(size_t i = 0; i < 12; i++)
    {
        std::vector<std::string>::iterator itr = std::find(joint_names_.begin(), joint_names_.end(), msg->name[i]);

        int index = std::distance(joint_names_.begin(), itr);
        current_joint_positions[index] = msg->position[i];
    }

    base_.updateJointPositions(current_joint_positions);
}

void StateEstimation::footContactCallback_(const champ_msgs::Contacts::ConstPtr& msg)
{
    for(size_t i = 0; i < 4; i++)
    {
        base_.legs[i]->gait_phase(msg->contacts[i]);
    }
}

void StateEstimation::publishVelocities_(const ros::TimerEvent& event)
{
    odometry_.getVelocities(current_velocities_);

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
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_footprint_frame_;

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

visualization_msgs::Marker StateEstimation::createMarker(geometry::Transformation foot_pos, int id, std::string frame_id)
{
    visualization_msgs::Marker foot_marker;

    foot_marker.header.frame_id = frame_id;

    foot_marker.type = visualization_msgs::Marker::SPHERE;
    foot_marker.action = visualization_msgs::Marker::ADD;
    foot_marker.id = id;

    foot_marker.pose.position.x = foot_pos.X();
    foot_marker.pose.position.y = foot_pos.Y();
    foot_marker.pose.position.z = foot_pos.Z();
    
    foot_marker.pose.orientation.x = 0.0;
    foot_marker.pose.orientation.y = 0.0;
    foot_marker.pose.orientation.z = 0.0;
    foot_marker.pose.orientation.w = 1.0;

    foot_marker.scale.x = 0.025;
    foot_marker.scale.y = 0.025;
    foot_marker.scale.z = 0.025;

    foot_marker.color.r = 0.780;
    foot_marker.color.g = 0.082;
    foot_marker.color.b = 0.521;
    foot_marker.color.a = 0.5;

    return foot_marker;
}

void StateEstimation::publishFootPositions_(const ros::TimerEvent& event)
{
    base_.getFootPositions(current_foot_positions_);

    visualization_msgs::MarkerArray marker_array;
    float robot_height;

    for(size_t i = 0; i < 4; i++)
    {
        marker_array.markers.push_back(createMarker(current_foot_positions_[i], i, base_link_frame_));
        robot_height += current_foot_positions_[i].Z();
    }

	if(foot_publisher_.getNumSubscribers())
    {
        foot_publisher_.publish(marker_array);
    }

    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();

    transformStamped.header.frame_id = base_footprint_frame_;
    transformStamped.child_frame_id = base_link_frame_;

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = -(robot_height / 4);

    tf2::Quaternion quaternion;
    quaternion.setRPY(0,0,0);

    //for now, only do pose estimation when the robot is static
    //until a proper foot detection is available
    //this is purely for visualization purpose

    //TODO: include this pose estimation approach inside libchamp
    if(current_velocities_.linear.x == 0.0 &&
       current_velocities_.linear.y == 0.0 &&
       current_velocities_.angular.z == 0.0)
    {
        //create orthonormal vectors
        tf2::Vector3 x_axis(current_foot_positions_[1].X() - current_foot_positions_[3].X(), 
                            current_foot_positions_[1].Y() - current_foot_positions_[3].Y(), 
                            current_foot_positions_[1].Z() - current_foot_positions_[3].Z());
        x_axis.normalize();

        tf2::Vector3 y_axis(current_foot_positions_[2].X() - current_foot_positions_[3].X(), 
                            current_foot_positions_[2].Y() - current_foot_positions_[3].Y(), 
                            current_foot_positions_[2].Z() - current_foot_positions_[3].Z());
        y_axis.normalize();

        //create a perpendicular vector 
        tf2::Vector3 z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        tf2::Matrix3x3 rotationMatrix(
                                x_axis.x(), y_axis.x(), z_axis.x(),
                                x_axis.y(), y_axis.y(), z_axis.y(),
                                x_axis.z(), y_axis.z(), z_axis.z());

        rotationMatrix.getRotation(quaternion);
        quaternion.normalize();
    }

    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = quaternion.z();
    transformStamped.transform.rotation.w = -quaternion.w();

    base_broadcaster_.sendTransform(transformStamped);    
}