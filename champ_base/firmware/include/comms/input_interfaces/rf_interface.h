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

#ifndef RF_INTERFACE_H
#define RF_INTERFACE_H

#include <Arduino.h>
#include <macros/macros.h>

namespace champ
{
    namespace Interfaces
    {
        class RF
        {
            int ele_, rud_, ail_, thr_, aux1_, aux2_;
            int lin_x_inv_, lin_y_inv_, ang_z_inv_, roll_inv_, pitch_inv_, yaw_inv_;
            int inverters_[6];

            champ::Velocities velocities_commands_;
            champ::Pose pose_commands_;

            unsigned long prev_vel_time_;
            unsigned long prev_pose_time_;
            unsigned long prev_poll_time_;

            bool vel_cmd_active_;
            bool pose_cmd_active_;
            public:
                RF(int ele, int rud, int ail, int thr, int aux1, int aux2):
                    ele_(ele),
                    rud_(rud),
                    ail_(ail),
                    thr_(thr),
                    aux1_(aux1),
                    aux2_(aux2),
                    vel_cmd_active_(false),
                    pose_cmd_active_(false)
                {
                }

                void invertAxes(bool linear_x, bool linear_y, bool angular_z, bool roll, bool pitch, bool yaw)
                {
                    lin_x_inv_ = (linear_x) ? -1 : 1;
                    lin_y_inv_ = (linear_y) ? -1 : 1;
                    ang_z_inv_ = (angular_z) ? -1 : 1;
                    roll_inv_ = (roll) ? -1 : 1;
                    pitch_inv_ = (pitch) ? -1 : 1;
                    yaw_inv_ = (yaw) ? -1 : 1;
                }

                void velocityInput(champ::Velocities &velocities)
                {
                    velocities = velocities_commands_;
                }

                void poseInput(champ::Pose &pose)
                {  
                    pose.orientation.roll = pose_commands_.orientation.roll;
                    pose.orientation.pitch = pose_commands_.orientation.pitch;
                    pose.orientation.yaw = pose_commands_.orientation.yaw;                
                }

                void jointsInput(float joints[12])
                {
     
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
                    return false;           
                }     

                void run()
                {
                    unsigned long now = micros();
                    if((now - prev_poll_time_) > 500000)
                    {
                        prev_poll_time_ = now;

                        vel_cmd_active_ = false;
                        pose_cmd_active_ = false;
                        
                        int rec_aux = pulseIn(aux1_, HIGH, 20000);
                        int rec_ele = pulseIn(ele_, HIGH, 20000);
                        int rec_rud = pulseIn(rud_, HIGH, 20000);
                        int rec_ail = pulseIn(ail_, HIGH, 20000);
                        // float rec_thr = pulseIn(thr_, HIGH, 20000);
                        // float rec_aux2 = pulseIn(aux2_, HIGH, 20000);

                        if((rec_ele > 1350 && rec_ele < 1650) &&
                            (rec_rud > 1350 && rec_rud < 1650) && 
                            (rec_ail > 1350 && rec_ail < 1650))
                        {
                            velocities_commands_.linear.x = 0.0;
                            velocities_commands_.linear.y = 0.0;
                            velocities_commands_.angular.z = 0.0;
                          
                            pose_commands_.orientation.roll = 0.0;
                            pose_commands_.orientation.pitch = 0.0;
                            pose_commands_.orientation.yaw = 0.0;

                            vel_cmd_active_ = false;
                            pose_cmd_active_ = false;
                        }
                        else if(rec_aux == 0)
                        {
                            velocities_commands_.linear.x = 0.0;
                            velocities_commands_.linear.y = 0.0;
                            velocities_commands_.angular.z = 0.0;

                            pose_commands_.orientation.roll = 0.0;
                            pose_commands_.orientation.pitch = 0.0;
                            pose_commands_.orientation.yaw = 0.0;

                            vel_cmd_active_ = false;
                            pose_cmd_active_ = false;
                        }
                        else if(1000 < rec_aux && rec_aux < 1500)
                        {
                            velocities_commands_.linear.x = mapFloat(rec_ele, 1100, 1900, -0.5, 0.5) * lin_x_inv_;
                            velocities_commands_.linear.y = mapFloat(rec_rud, 1100, 1900, -0.5, 0.5) * lin_y_inv_;
                            velocities_commands_.angular.z = mapFloat(rec_ail, 1100, 1900, -1.0, 1.0) * ang_z_inv_;
                            
                            vel_cmd_active_ = true;
                            pose_cmd_active_ = false;
                        }
                        else if(1500 < rec_aux && rec_aux < 2000)
                        {
                            pose_commands_.orientation.roll = mapFloat(rec_ail, 1100, 1900, -0.523599, 0.523599) * roll_inv_;
                            pose_commands_.orientation.pitch = mapFloat(rec_ele, 1100, 1900, -0.314159, 0.314159) * pitch_inv_;
                            pose_commands_.orientation.yaw = mapFloat(rec_rud, 1100, 1900, -0.436332, 0.436332) *yaw_inv_;
                            
                            vel_cmd_active_ = false;
                            pose_cmd_active_ = true;
                        }
                    }
                }
        };
    } 
}

#endif 