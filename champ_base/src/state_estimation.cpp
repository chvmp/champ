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

champ::Odometry::Time rosTimeToChampTime(const ros::Time& time)
{
  return time.toNSec() / 1000ul;
}

StateEstimation::StateEstimation(ros::NodeHandle *nh, ros::NodeHandle *pnh):
    odometry_(base_, rosTimeToChampTime(ros::Time::now()))
{
    joint_states_subscriber_.subscribe(*nh, "joint_states", 1);
    foot_contacts_subscriber_ .subscribe(*nh, "foot_contacts", 1);

    sync.reset(new Sync(SyncPolicy(10), joint_states_subscriber_, foot_contacts_subscriber_));
    sync->registerCallback(boost::bind(&StateEstimation::synchronized_callback_, this, _1, _2));
    
    footprint_to_odom_publisher_  = nh->advertise<nav_msgs::Odometry>("odom/raw", 1);
    base_to_footprint_publisher_  = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("base_to_footprint_pose", 1);
    foot_publisher_   = nh->advertise<visualization_msgs::MarkerArray>("foot", 1);

    nh->getParam("links_map/base", base_name_);
    nh->getParam("gait/odom_scaler", gait_config_.odom_scaler);
    pnh->param("orientation_from_imu", orientation_from_imu_, false);

    if (orientation_from_imu_)
      imu_subscriber_ = nh->subscribe<sensor_msgs::Imu>("imu/data", 1, &StateEstimation::imu_callback_, this);

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
                                        &StateEstimation::publishFootprintToOdom_, 
                                        this);

    base_pose_timer_ = pnh->createTimer(ros::Duration(0.02),
                                            &StateEstimation::publishBaseToFootprint_, 
                                            this);
}

void StateEstimation::synchronized_callback_(const sensor_msgs::JointStateConstPtr& joints_msg, const champ_msgs::ContactsStampedConstPtr& contacts_msg)
{
    last_sync_time_ = ros::Time::now();

    float current_joint_positions[12];

    for(size_t i = 0; i < joints_msg->name.size(); i++)
    {
        std::vector<std::string>::iterator itr = std::find(joint_names_.begin(), joint_names_.end(), joints_msg->name[i]);

        int index = std::distance(joint_names_.begin(), itr);
        current_joint_positions[index] = joints_msg->position[i];
    }

    base_.updateJointPositions(current_joint_positions);

    for(size_t i = 0; i < 4; i++)
    {
        base_.legs[i]->in_contact(contacts_msg->contacts[i]);
    }
}

void StateEstimation::imu_callback_(const sensor_msgs::ImuConstPtr& msg)
{
  last_imu_ = msg;
}

void StateEstimation::publishFootprintToOdom_(const ros::TimerEvent& event)
{
    odometry_.getVelocities(current_velocities_, rosTimeToChampTime(ros::Time::now()));

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
    odom.pose.covariance[0] = 0.25;
    odom.pose.covariance[7] = 0.25;
    odom.pose.covariance[35] = 0.017;

    odom.twist.twist.linear.x = current_velocities_.linear.x;
    odom.twist.twist.linear.y = current_velocities_.linear.y;
    odom.twist.twist.linear.z = 0.0;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = current_velocities_.angular.z;

    odom.twist.covariance[0] = 0.3;
    odom.twist.covariance[7] = 0.3;
    odom.twist.covariance[35] = 0.017;
    
    footprint_to_odom_publisher_.publish(odom);
}

visualization_msgs::Marker StateEstimation::createMarker_(geometry::Transformation foot_pos, int id, std::string frame_id)
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

void StateEstimation::publishBaseToFootprint_(const ros::TimerEvent& event)
{
    base_.getFootPositions(current_foot_positions_);

    visualization_msgs::MarkerArray marker_array;
    float robot_height = 0.0, all_height = 0.0;
    int foot_in_contact = 0;
    geometry::Transformation touching_feet[4];
    bool no_contact = false;

    for(size_t i = 0; i < 4; i++)
    {
        marker_array.markers.push_back(createMarker_(current_foot_positions_[i], i, base_link_frame_));
        if(base_.legs[i]->in_contact())
        {
            robot_height += current_foot_positions_[i].Z();
            touching_feet[foot_in_contact] = current_foot_positions_[i];
            foot_in_contact++;
        }
        all_height += current_foot_positions_[i].Z();
    }

    if (foot_in_contact == 0)
    {
      no_contact = true;
      robot_height = all_height;
      foot_in_contact = 4;
      for (size_t i = 0; i < 4; ++i)
        touching_feet[i] = current_foot_positions_[i];
    }

	if(foot_publisher_.getNumSubscribers())
    {
        foot_publisher_.publish(marker_array);
    }

    tf2::Vector3 x_axis(1, 0, 0);
    tf2::Vector3 y_axis(0, 1, 0);
    tf2::Vector3 z_axis(0, 0, 1);

    // if the IMU provides good orientation estimates, these can be used to
    // greatly improve body orientation; IMUs in Gazebo provide even non-noisy
    // orientation measurements!
    tf2::Matrix3x3 imu_rotation;
    if (orientation_from_imu_ && last_imu_ != nullptr)
    {
      tf2::Quaternion imu_orientation(
        last_imu_->orientation.x,
        last_imu_->orientation.y,
        last_imu_->orientation.z,
        last_imu_->orientation.w);
      imu_rotation.setRotation(imu_orientation);
    }
    else
    {
      imu_rotation.setIdentity();
    }

    // handle the orientation estimation based on the number of touching legs
    if (foot_in_contact >= 3 && !no_contact)
    {
        // 3 or 4 legs touching. 3 points are enough to form a plane, so we choose
        // any 3 touching legs and create a plane from them

        // create two vectors in base_footprint plane
        x_axis = tf2::Vector3(touching_feet[0].X() - touching_feet[2].X(),
                              touching_feet[0].Y() - touching_feet[2].Y(),
                              touching_feet[0].Z() - touching_feet[2].Z());
        x_axis.normalize();

        y_axis = tf2::Vector3(touching_feet[1].X() - touching_feet[2].X(),
                              touching_feet[1].Y() - touching_feet[2].Y(),
                              touching_feet[1].Z() - touching_feet[2].Z());
        y_axis.normalize();

        // compute normal vector of the plane
        z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        // we don't know which 3 feet were chosen, so it might happen the normal points downwards
        if (z_axis.dot(tf2::Vector3(0, 0, 1)) < 0)
          z_axis = -z_axis;

        // project 0,1,0 base_link axis to the plane defined by the normal
        y_axis = (tf2::Vector3(0, 1, 0) - (tf2::Vector3(0, 1, 0).dot(z_axis) * z_axis)).normalized();
        // and find the last vector which just has to be perpendicular to y and z
        x_axis = y_axis.cross(z_axis);
    }
    else if (foot_in_contact == 2)
    {
      if ((base_.legs[0]->in_contact() && base_.legs[2]->in_contact()) ||
          (base_.legs[1]->in_contact() && base_.legs[3]->in_contact()))
      {
        // both left or both right legs are touching... let them define the x axis
        x_axis = tf2::Vector3(touching_feet[0].X() - touching_feet[1].X(),
                              touching_feet[0].Y() - touching_feet[1].Y(),
                              touching_feet[0].Z() - touching_feet[1].Z());
        x_axis.normalize();

        // get Z from IMU as we do not have enough contact points to define a plane
        z_axis = imu_rotation.inverse() * z_axis;
        y_axis = z_axis.cross(x_axis);
        // and find the last vector which just has to be perpendicular to y and z
        x_axis = y_axis.cross(z_axis);
      }
      else if ((base_.legs[0]->in_contact() && base_.legs[1]->in_contact()) ||
               (base_.legs[2]->in_contact() && base_.legs[3]->in_contact()))
      {
        // both front or both hind legs are touching... let them define the y axis
        y_axis = tf2::Vector3(touching_feet[0].X() - touching_feet[1].X(),
                              touching_feet[0].Y() - touching_feet[1].Y(),
                              touching_feet[0].Z() - touching_feet[1].Z());
        y_axis.normalize();

        // get Z from IMU as we do not have enough contact points to define a plane
        z_axis = imu_rotation.inverse() * z_axis;
        x_axis = y_axis.cross(z_axis);
        // and find the last vector which just has to be perpendicular to x and z
        y_axis = z_axis.cross(x_axis);
      }
      else
      {
        // diagonal legs touching... axis1 is the line going through both touching
        // legs. axis2 is perpendicular to axis1 and z axis (from IMU)... then we
        // just rotate axis1 and axis2 to form a coordinate system
        tf2::Vector3 axis1(touching_feet[0].X() - touching_feet[1].X(),
                           touching_feet[0].Y() - touching_feet[1].Y(),
                           touching_feet[0].Z() - touching_feet[1].Z());
        axis1.normalize();

        // get Z from IMU as we do not have enough contact points to define a plane
        z_axis = imu_rotation.inverse() * z_axis;
        auto axis2 = z_axis.cross(axis1);
        z_axis = axis1.cross(axis2);

        // project base_link 1,0,0 axis along the computed plane normal
        x_axis = (x_axis - (x_axis.dot(z_axis) * z_axis)).normalized();
        // and find the last vector which just has to be perpendicular to x and z
        y_axis = z_axis.cross(x_axis);
      }
    }
    else if (foot_in_contact == 1 || no_contact)
    {
      // Zero or one feet in contact... There isn't much to do, so just take Z from IMU
      z_axis = imu_rotation.inverse() * z_axis;

      // project base_link 1,0,0 axis along the computed plane normal
      x_axis = (x_axis - (x_axis.dot(z_axis) * z_axis)).normalized();
      // and find the last vector which just has to be perpendicular to x and z
      y_axis = z_axis.cross(x_axis);
    }

    tf2::Matrix3x3 rotationMatrix(
      x_axis.x(), y_axis.x(), z_axis.x(),
      x_axis.y(), y_axis.y(), z_axis.y(),
      x_axis.z(), y_axis.z(), z_axis.z());

    tf2::Quaternion quaternion;
    rotationMatrix.getRotation(quaternion);
    quaternion.normalize();

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = base_footprint_frame_;
    pose_msg.header.stamp = ros::Time::now();

    pose_msg.pose.covariance[0] = 0.001;
    pose_msg.pose.covariance[7] = 0.001;
    pose_msg.pose.covariance[14] = 0.001;
    pose_msg.pose.covariance[21] = 0.0001;
    pose_msg.pose.covariance[28] = 0.0001;
    pose_msg.pose.covariance[35] = 0.017;

    pose_msg.pose.pose.position.x = 0.0;
    pose_msg.pose.pose.position.y = 0.0;
    pose_msg.pose.pose.position.z = -(robot_height / (float)foot_in_contact);

    pose_msg.pose.pose.orientation.x = quaternion.x();
    pose_msg.pose.pose.orientation.y = quaternion.y();
    pose_msg.pose.pose.orientation.z = quaternion.z();
    pose_msg.pose.pose.orientation.w = -quaternion.w();

    base_to_footprint_publisher_.publish(pose_msg);
}