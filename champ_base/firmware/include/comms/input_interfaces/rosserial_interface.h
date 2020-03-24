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

#ifndef ROSSERIAL_INTERFACE_H
#define ROSSERIAL_INTERFACE_H

#include <ros.h>
#include <ros/time.h>
#include <Arduino.h>
#include <geometry/geometry.h>
#include <quadruped_base/quadruped_components.h>
#include <geometry_msgs/Twist.h>
#include <champ_msgs/Point.h>
#include <champ_msgs/PointArray.h>
#include <champ_msgs/Joints.h>
#include <champ_msgs/Pose.h>
#include <champ_msgs/Imu.h>
#include <champ_msgs/Velocities.h>

namespace champ
{
    namespace Interfaces
    {
        class ROSSerial
        {
            ros::NodeHandle nh_;

            ros::Subscriber<geometry_msgs::Twist, ROSSerial> vel_cmd_sub_;
            ros::Subscriber<champ_msgs::Pose, ROSSerial> pose_cmd_sub_;
            ros::Subscriber<champ_msgs::Joints, ROSSerial> joints_cmd_sub_;

            champ_msgs::PointArray point_msg_;
            ros::Publisher point_pub_;

            champ_msgs::Joints joints_msg_;
            ros::Publisher jointstates_pub_;

            champ_msgs::Imu imu_msg_;
            ros::Publisher imu_pub_;

            champ_msgs::Velocities vel_msg_;
            ros::Publisher vel_pub_;

            champ::Velocities *velocities_commands_;
            champ::Pose *pose_commands_;
            float joints_commands_[12];

            unsigned long *prev_vel_time_;
            unsigned long *prev_pose_time_;
            unsigned long *prev_joints_time_;
            unsigned long prev_resetter_time_;

            bool vel_cmd_active_;
            bool joints_cmd_active_;
            bool pose_cmd_active_;

            void velocityCommandCallback(const geometry_msgs::Twist& vel_cmd_msg)
            {
                *prev_vel_time_ = micros();
      
                velocities_commands_->linear.x = vel_cmd_msg.linear.x;
                velocities_commands_->linear.y = vel_cmd_msg.linear.y;
                velocities_commands_->angular.z = vel_cmd_msg.angular.z;
            }

            void poseCommandCallback(const champ_msgs::Pose& pose_cmd_msg)
            {
                *prev_pose_time_ = micros();

                pose_commands_->orientation.roll = pose_cmd_msg.roll;
                pose_commands_->orientation.pitch = pose_cmd_msg.pitch;
                pose_commands_->orientation.yaw = pose_cmd_msg.yaw;
                pose_commands_->translation.z = pose_cmd_msg.z;
            }

            void jointsCommandCallback(const champ_msgs::Joints& joints_cmd_msg)
            {
                *prev_joints_time_ = micros();

                for(unsigned int i = 0; i < 12; i++)
                {
                    joints_commands_[i] = joints_cmd_msg.position[i];
                }
            }

            public:
                ROSSerial():
                    vel_cmd_sub_("champ/cmd_vel", &ROSSerial::velocityCommandCallback, this),
                    pose_cmd_sub_("champ/cmd_pose", &ROSSerial::poseCommandCallback, this),
                    joints_cmd_sub_("champ/cmd_joints", &ROSSerial::jointsCommandCallback, this),
                    point_pub_("/champ/foot/raw", &point_msg_),
                    jointstates_pub_("/champ/joint_states/raw", &joints_msg_),
                    imu_pub_("/champ/imu/raw", &imu_msg_),
                    vel_pub_("/champ/velocities/raw", &vel_msg_),
                    vel_cmd_active_(false),
                    joints_cmd_active_(false),
                    pose_cmd_active_(false)

                {
                    joints_msg_.position_length = 12;

                    unsigned long now1 = micros();
                    unsigned long now2 = micros();
                    unsigned long now3 = micros();

                    prev_vel_time_ = &now1;
                    prev_pose_time_ = &now2;
                    prev_joints_time_ = &now3;

                    nh_.initNode();
                    nh_.getHardware()->setBaud(500000);

                    nh_.subscribe(vel_cmd_sub_);
                    nh_.subscribe(pose_cmd_sub_);
                    nh_.subscribe(joints_cmd_sub_);

                    nh_.advertise(point_pub_);
                    nh_.advertise(jointstates_pub_);
                    nh_.advertise(imu_pub_);
                    nh_.advertise(vel_pub_);

                    // while (!nh_.connected())
                    // {
                    //     nh_.spinOnce();
                    // }
                    nh_.loginfo("CHAMP ROS CLIENT CONNECTED");
                }

                void velocityInput(champ::Velocities &velocities)
                {
                    velocities = *velocities_commands_;
                }

                void poseInput(champ::Pose &pose)
                {  
                    pose = *pose_commands_;
                }

                void jointsInput(float joints[12])
                {
                    for(unsigned int i = 0; i < 12; i++)
                    {
                        joints[i] = joints_commands_[i];
                    }            
                }

                bool velInputIsActive()
                {
                    return vel_cmd_active_;
                }

                bool poseInputIsActive()
                {       
                    return pose_cmd_active_;    
                }

                bool jointsInputIsActive()
                {
                    return joints_cmd_active_;
                }

                void run()
                {   
                    unsigned long now = micros();
                    if((now- prev_resetter_time_) > 333333)
                    {
                        prev_resetter_time_ = now;
                        if((now - *prev_vel_time_) < 500000)
                        {
                            vel_cmd_active_ = true;
                        }
                        else
                        {
                            vel_cmd_active_ = false;

                            velocities_commands_->linear.x = 0.0f;
                            velocities_commands_->linear.y = 0.0f;
                            velocities_commands_->angular.z = 0.0f;
                        }

                        if((now - *prev_pose_time_) < 500000)
                        {
                            pose_cmd_active_ = true;
                        }
                        else
                        {   
                            pose_cmd_active_ = false;

                            pose_commands_->orientation.roll = 0.0f;
                            pose_commands_->orientation.pitch = 0.0f;
                            pose_commands_->orientation.yaw = 0.0f;
                        }

                        if((now - *prev_joints_time_) < 500000)
                        {
                            joints_cmd_active_ = true;
                        }
                        else
                        {
                            joints_cmd_active_ = false;
                        }
                    }
                    nh_.spinOnce();
                }
                
                void publishPoints(geometry::Transformation foot_positions[4])
                {
                    point_msg_.lf.x = foot_positions[0].X();
                    point_msg_.lf.y = foot_positions[0].Y();
                    point_msg_.lf.z = foot_positions[0].Z();

                    point_msg_.rf.x = foot_positions[1].X();
                    point_msg_.rf.y = foot_positions[1].Y();
                    point_msg_.rf.z = foot_positions[1].Z();

                    point_msg_.lh.x = foot_positions[2].X();
                    point_msg_.lh.y = foot_positions[2].Y();
                    point_msg_.lh.z = foot_positions[2].Z();

                    point_msg_.rh.x = foot_positions[3].X();
                    point_msg_.rh.y = foot_positions[3].Y();
                    point_msg_.rh.z = foot_positions[3].Z();

                    point_pub_.publish(&point_msg_);
                }

                void publishVelocities(champ::Velocities vel)
                {
                    vel_msg_.linear_x = vel.linear.x;
                    vel_msg_.linear_y = vel.linear.y;
                    vel_msg_.angular_z = vel.angular.z;

                    vel_pub_.publish(&vel_msg_);
                }

                void publishJointStates(float joint_positions[12])
                {
                    joints_msg_.position = joint_positions;
                    jointstates_pub_.publish(&joints_msg_);  
                }

                void publishIMU(champ::Quaternion &orientation, champ::Accelerometer &accel, champ::Gyroscope &gyro, champ::Magnetometer &mag)
                {
                    imu_msg_.orientation.w = orientation.w;
                    imu_msg_.orientation.x = orientation.x;
                    imu_msg_.orientation.y = orientation.y;
                    imu_msg_.orientation.z = orientation.z;

                    imu_msg_.linear_acceleration.x = accel.x;
                    imu_msg_.linear_acceleration.y = accel.y;
                    imu_msg_.linear_acceleration.z = accel.z;

                    imu_msg_.angular_velocity.x = gyro.x;
                    imu_msg_.angular_velocity.y = gyro.y;
                    imu_msg_.angular_velocity.z = gyro.z;

                    imu_msg_.magnetic_field.x = mag.x;
                    imu_msg_.magnetic_field.y = mag.y;
                    imu_msg_.magnetic_field.z = mag.z;

                    imu_pub_.publish(&imu_msg_);
                }
        };
    }
    
}
#endif