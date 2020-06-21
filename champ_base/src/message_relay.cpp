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

#include <message_relay.h>

MessageRelay::MessageRelay(const ros::NodeHandle &node_handle,
                           const ros::NodeHandle &private_node_handle):
    nh_(node_handle),
    pnh_(private_node_handle)
{    
    imu_data_.orientation.w = 1.0;

    foot_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("foot", 100);
    imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 100);
    mag_publisher_ = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 100);
    joint_states_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
    joint_commands_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 100);
    odometry_publisher_  = nh_.advertise<nav_msgs::Odometry>("odom/raw", 100);

    cmd_pose_subscriber_ = nh_.subscribe("cmd_pose", 1, &MessageRelay::cmdPoseCallback_, this);
    foot_raw_subscriber_ = nh_.subscribe("foot/raw", 1, &MessageRelay::footRawCallback, this);
    imu_raw_subscriber_ = nh_.subscribe("imu/raw", 1, &MessageRelay::IMURawCallback, this);
    joints_raw_subscriber_ = nh_.subscribe("joint_states/raw", 1, &MessageRelay::jointStatesRawCallback, this);
    velocities_raw_subscriber_ = nh_.subscribe("velocities/raw", 1, &MessageRelay::odometryRawCallback, this);

    nh_.getParam("links_map/base", base_name_);
    pnh_.getParam("gazebo",        in_gazebo_);
    pnh_.getParam("has_imu",       has_imu_);

    joint_names_ = champ::URDF::getJointNames(nh_);

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
    imu_frame_ = node_namespace_ + "imu_link";
    base_link_frame_ = node_namespace_ + base_name_;
}

void MessageRelay::footRawCallback(const champ_msgs::PointArray::ConstPtr& msg)
{
    visualization_msgs::MarkerArray marker_array;

    geometry::Transformation temp_foot_pos;

    temp_foot_pos.X() = msg->lf.x;
    temp_foot_pos.Y() = msg->lf.y;
    temp_foot_pos.Z() = msg->lf.z;
    marker_array.markers.push_back(createMarker(temp_foot_pos, 0, base_link_frame_));

    temp_foot_pos.X() = msg->rf.x;
    temp_foot_pos.Y() = msg->rf.y;
    temp_foot_pos.Z() = msg->rf.z;
    marker_array.markers.push_back(createMarker(temp_foot_pos, 1, base_link_frame_));

    temp_foot_pos.X() = msg->lh.x;
    temp_foot_pos.Y() = msg->lh.y;
    temp_foot_pos.Z() = msg->lh.z;
    marker_array.markers.push_back(createMarker(temp_foot_pos, 2, base_link_frame_));

    temp_foot_pos.X() = msg->rh.x;
    temp_foot_pos.Y() = msg->rh.y;
    temp_foot_pos.Z() = msg->rh.z;
    marker_array.markers.push_back(createMarker(temp_foot_pos, 3, base_link_frame_));

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = base_footprint_frame_;
    transformStamped.child_frame_id = base_link_frame_;

    float robot_height =  (msg->lf.z + msg->rf.z + msg->lh.z + msg->rh.z) / 4.0;

    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = -robot_height;

    //TODO:: do a proper pose estimation to get RPY
    if(has_imu_)
    {
        tf2::Quaternion temp_quat(
            imu_data_.orientation.x,
            imu_data_.orientation.y,
            imu_data_.orientation.z,
            imu_data_.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3 temp_m(temp_quat);
        temp_m.getRPY(roll, pitch, yaw);

        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, 0);

        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();
    }
    else
    {
        tf2::Quaternion quaternion;
        quaternion.setRPY(0,0,0);
        
        if(current_velocities_.linear.x == 0.0 &&
           current_velocities_.linear.y == 0.0 &&
           current_velocities_.angular.z == 0.0)
        {
            // ROS_INFO("HELLO");
            //create orthonormal vectors
            tf2::Vector3 x_axis(msg->rf.x - msg->rh.x, 
                                msg->rf.y - msg->rh.y, 
                                msg->rf.z - msg->rh.z);
            x_axis.normalize();

            tf2::Vector3 y_axis(msg->lh.x - msg->rh.x, 
                                msg->lh.y - msg->rh.y, 
                                msg->lh.z - msg->rh.z);
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
    }

	if(foot_publisher_.getNumSubscribers())
    {
        foot_publisher_.publish(marker_array);
    }
    base_broadcaster_.sendTransform(transformStamped);
}

void MessageRelay::IMURawCallback(const champ_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu imu_data_msg;
    sensor_msgs::MagneticField imu_mag_msg;

    imu_data_msg.header.stamp = ros::Time::now();
    imu_data_msg.header.frame_id = imu_frame_;

    imu_data_msg.orientation.w = msg->orientation.w;
    imu_data_msg.orientation.x = msg->orientation.x;
    imu_data_msg.orientation.y = msg->orientation.y;
    imu_data_msg.orientation.z = msg->orientation.z;

    imu_data_msg.linear_acceleration.x = msg->linear_acceleration.x;
    imu_data_msg.linear_acceleration.y = msg->linear_acceleration.y;
    imu_data_msg.linear_acceleration.z = msg->linear_acceleration.z;

    imu_data_msg.angular_velocity.x = msg->angular_velocity.x;
    imu_data_msg.angular_velocity.y = msg->angular_velocity.y;
    imu_data_msg.angular_velocity.z = msg->angular_velocity.z;

    imu_data_msg.orientation_covariance[0] = 0.0025;
    imu_data_msg.orientation_covariance[4] = 0.0025;
    imu_data_msg.orientation_covariance[8] = 0.0025;

    imu_data_msg.angular_velocity_covariance[0] = 0.000001;
    imu_data_msg.angular_velocity_covariance[4] = 0.000001;
    imu_data_msg.angular_velocity_covariance[8] = 0.000001;

    imu_data_msg.linear_acceleration_covariance[0] = 0.0001;
    imu_data_msg.linear_acceleration_covariance[4] = 0.0001;
    imu_data_msg.linear_acceleration_covariance[8] = 0.0001;

    imu_data_ = imu_data_msg;

    //prevent from clashing in gazebo imu
    if(!has_imu_)
    {
        return;
    }

    imu_publisher_.publish(imu_data_msg);

    imu_mag_msg.header.stamp = ros::Time::now();
    imu_mag_msg.header.frame_id = imu_frame_;

    imu_mag_msg.magnetic_field.x = msg->magnetic_field.x;
    imu_mag_msg.magnetic_field.y = msg->magnetic_field.y;
    imu_mag_msg.magnetic_field.z = msg->magnetic_field.z;

    imu_mag_msg.magnetic_field_covariance[0] = 0.000001;
    imu_mag_msg.magnetic_field_covariance[4] = 0.000001;
    imu_mag_msg.magnetic_field_covariance[8] = 0.000001;

    mag_publisher_.publish(imu_mag_msg);
}

void MessageRelay::jointStatesRawCallback(const champ_msgs::Joints::ConstPtr& msg)
{
    if(in_gazebo_)
    {
        trajectory_msgs::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.header.stamp = ros::Time::now();
        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(12);

        point.time_from_start = ros::Duration(1.0 / 60.0);
        for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = msg->position[i];
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
            joints_msg.position[i]= msg->position[i];
        }

        joint_states_publisher_.publish(joints_msg);
    }
}

void MessageRelay::odometryRawCallback(const champ_msgs::Velocities::ConstPtr& msg)
{
    current_velocities_.linear.x = msg->linear_x;
    current_velocities_.linear.y = msg->linear_y;
    current_velocities_.angular.z = msg->angular_z;

    ros::Time current_time = ros::Time::now();  

    double vel_dt = (current_time - last_vel_time_).toSec();
    last_vel_time_ = current_time;

    //rotate in the z axis
    //https://en.wikipedia.org/wiki/Rotation_matrix
    double delta_heading = msg->angular_z * vel_dt; 
    double delta_x = (msg->linear_x * cos(heading_) - msg->linear_y * sin(heading_)) * vel_dt; //m
    double delta_y = (msg->linear_x * sin(heading_) + msg->linear_y * cos(heading_)) * vel_dt; //m

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

    odom.twist.twist.linear.x = msg->linear_x;
    odom.twist.twist.linear.y = msg->linear_y;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = msg->angular_z;

    odom.twist.covariance[0] = 0.001;
    odom.twist.covariance[7] = 0.001;
    odom.twist.covariance[35] = 0.001;

    odometry_publisher_.publish(odom);
}

visualization_msgs::Marker MessageRelay::createMarker(geometry::Transformation foot_pos, int id, std::string frame_id)
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

void MessageRelay::cmdPoseCallback_(const champ_msgs::Pose::ConstPtr& msg)
{
    req_pose_.orientation.roll = msg->roll;
    req_pose_.orientation.pitch = msg->pitch;
    req_pose_.orientation.yaw = msg->yaw;
}