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

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <quadruped_base/quadruped_base.h>
#include <geometry/geometry.h>

namespace champ
{
    class Odometry
    {
        QuadrupedBase *base_;
        geometry::Transformation prev_foot_position_[4];
        bool prev_gait_phase_[4];
        float prev_theta_[4];
        unsigned long int prev_time_;
        int theta_direction_[4];

        public:
            Odometry(QuadrupedBase &quadruped_base):
                base_(&quadruped_base),
                prev_gait_phase_{1,1,1,1},
                prev_theta_{0,0,0,0},
                prev_time_(0),
                theta_direction_{1,1,-1,-1}
            {
                for(unsigned int i = 0; i < 4; i++)
                {
                    prev_foot_position_[i] = base_->legs[i]->foot_from_hip();
                }
            }        
            
            void getVelocities(champ::Velocities &vel)
            {
                unsigned int total_contact = 0;
                float x_sum = 0;
                float y_sum = 0;
                float theta_sum = 0;

                for(unsigned int i = 0; i < 4; i++)
                {
                    geometry::Transformation current_foot_position = base_->legs[i]->foot_from_base();
                
                    bool current_gait_phase = base_->legs[i]->gait_phase();
                    
                    float delta_x = (prev_foot_position_[i].X() - current_foot_position.X());
                    float delta_y = (prev_foot_position_[i].Y() - current_foot_position.Y());
                    
                    float current_theta = atan2f(current_foot_position.Y(), current_foot_position.X());
                    float delta_theta = (prev_theta_[i] - current_theta);

                    if(current_gait_phase && prev_gait_phase_[i])
                    {
                        total_contact += 1;
                
                        x_sum += delta_x;
                        y_sum += delta_y;
                        theta_sum += delta_theta;
                    }

                    prev_foot_position_[i] = current_foot_position;
                    prev_gait_phase_[i] = current_gait_phase;
                    prev_theta_[i] = current_theta;
                }

                if(total_contact > 1)
                {
                    x_sum /= total_contact;
                    y_sum /= total_contact;
                    theta_sum /= total_contact;
                }

                unsigned long int now = time_us();
                double dt = (now - prev_time_) / 1000000.0;
                
                vel.linear.x = x_sum / dt;
                vel.linear.y = y_sum / dt;
                vel.angular.z = theta_sum / dt;
                
                prev_time_ = now;
            }
    };
}

#endif