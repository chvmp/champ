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

#include <quadruped_controller.h>

QuadrupedController::QuadrupedController(const ros::NodeHandle &node_handle,
                                         const ros::NodeHandle &private_node_handle):
    nh_(node_handle),
    pnh_(private_node_handle),
    body_controller_(base_),
    leg_controller_(base_),
    kinematics_(base_),
    odometry_(base_)
{
    cmd_vel_subscriber_ = nh_.subscribe("cmd_vel/smooth", 1, &QuadrupedController::cmdVelCallback_, this);
    cmd_pose_subscriber_ = nh_.subscribe("cmd_pose", 1, &QuadrupedController::cmdPoseCallback_, this);
    
    joint_states_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
    joint_commands_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>("joint_group_position_controller/command", 100);
    velocities_publisher_   = nh_.advertise<nav_msgs::Odometry>("odom/raw", 100);
    foot_publisher_   = nh_.advertise<visualization_msgs::MarkerArray>("foot", 100);

    std::string knee_orientation;
    nh_.getParam("gait/pantograph_leg",         gait_config_.pantograph_leg);
    nh_.getParam("gait/max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    nh_.getParam("gait/max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    nh_.getParam("gait/max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    nh_.getParam("gait/com_x_translation",      gait_config_.com_x_translation);
    nh_.getParam("gait/swing_height",           gait_config_.swing_height);
    nh_.getParam("gait/stance_depth",           gait_config_.stance_depth);
    nh_.getParam("gait/stance_duration",        gait_config_.stance_duration);
    nh_.getParam("gait/nominal_height",         gait_config_.nominal_height);
    nh_.getParam("gait/knee_orientation",       knee_orientation);
    nh_.getParam("links_map/base",              base_name_);
    pnh_.getParam("gazebo",                     in_gazebo_);
    gait_config_.knee_orientation = knee_orientation.c_str();
    
    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromServer(base_, nh_);
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
    base_link_frame_ = node_namespace_ + base_name_;

    tf2_ros::Buffer tfBuffer;
    tfBuffer.setUsingDedicatedThread(true);
    geometry_msgs::TransformStamped transformStamped;

    loop_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                   &QuadrupedController::controlLoop_,
                                   this);

    odom_data_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                        &QuadrupedController::publishVelocities_, 
                                        this);

    joints_position_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                              &QuadrupedController::publishJoints_, 
                                              this);

    foot_position_timer_ = pnh_.createTimer(ros::Duration(0.005),
                                            &QuadrupedController::publishFootPositions_, 
                                            this);

    req_pose_.position.z = gait_config_.nominal_height;
}

void QuadrupedController::controlLoop_(const ros::TimerEvent& event)
{
    float target_joint_positions[12];
    
    body_controller_.poseCommand(target_foot_positions_, req_pose_);
    leg_controller_.velocityCommand(target_foot_positions_, req_vel_);
    kinematics_.inverse(target_joint_positions, target_foot_positions_);

    //This is a pseudo actuator.Virtually this just stores the set target_joint_positions by the controller.
    //For physical hardware, the actuator class can be used to call physical hardware APIs
    //to set the actuators' angles using target_joint_positions.
    actuators_.moveJoints(target_joint_positions);
    
    //Virtually, this just grabs the stored target_joint_positions set by the controller.
    //On real hardware, the actuator class can be used to grab the physical actuators' angles
    //and return those values in getJointPositions function.
    actuators_.getJointPositions(current_joint_positions_);
    base_.updateJointPositions(current_joint_positions_);
    base_.getFootPositions(current_foot_positions_);
    odometry_.getVelocities(current_velocities_);
}

void QuadrupedController::cmdVelCallback_(const geometry_msgs::Twist::ConstPtr& msg)
{
    req_vel_.linear.x = msg->linear.x;
    req_vel_.linear.y = msg->linear.y;
    req_vel_.angular.z = msg->angular.z;
}

void QuadrupedController::cmdPoseCallback_(const champ_msgs::Pose::ConstPtr& msg)
{
    req_pose_.orientation.roll = msg->roll;
    req_pose_.orientation.pitch = msg->pitch;
    req_pose_.orientation.yaw = msg->yaw;

    req_pose_.position.z = msg->z * gait_config_.nominal_height;
    if(req_pose_.position.z < (gait_config_.nominal_height * 0.5))
        req_pose_.position.z = gait_config_.nominal_height * 0.5;
}

void QuadrupedController::publishJoints_(const ros::TimerEvent& event)
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
            point.positions[i] = current_joint_positions_[i];
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
            joints_msg.position[i]= current_joint_positions_[i];
        }

        joint_states_publisher_.publish(joints_msg);
    }
}

void QuadrupedController::publishVelocities_(const ros::TimerEvent& event)
{
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

visualization_msgs::Marker QuadrupedController::createMarker(geometry::Transformation foot_pos, int id, std::string frame_id)
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

void QuadrupedController::publishFootPositions_(const ros::TimerEvent& event)
{
    visualization_msgs::MarkerArray marker_array;
    float robot_height;

    for(size_t i = 0; i < 4; i++)
    {
        geometry::Transformation temp_foot_pos = target_foot_positions_[i];
        //the target foot position is calculated in the hip frame
        //now transform this to base so we can visualize in rviz
        champ::Kinematics::transformToBase(temp_foot_pos, *base_.legs[i]);
        marker_array.markers.push_back(createMarker(temp_foot_pos, i, base_link_frame_));
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
    transformStamped.transform.translation.z = -robot_height / 4;

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