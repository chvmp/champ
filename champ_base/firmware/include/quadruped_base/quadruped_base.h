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

#ifndef QUADRUPPED_BASE_H
#define QUADRUPPED_BASE_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>
#include <quadruped_base/quadruped_leg.h>
#include <quadruped_base/quadruped_joint.h>
#include <quadruped_base/quadruped_components.h>

namespace champ
{
    class QuadrupedBase
    {   
        Velocities speed_;
        
        int getKneeDirection(char direction)
        {
            switch (direction) 
            {
                case '>':
                    return -1;
                case '<':
                    return 1;
                default:
                    return -1;
            }
        }    

        public:
            QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg, GaitConfig &gait_conf):        
                lf(&lf_leg),
                rf(&rf_leg),
                lh(&lh_leg),
                rh(&rh_leg),
                gait_config(&gait_conf)
            {
                unsigned int total_legs = 0;

                legs[total_legs++] = lf;
                legs[total_legs++] = rf;
                legs[total_legs++] = lh;
                legs[total_legs++] = rh;

                for(unsigned int i=0; i < 4; i++)
                {
                    int dir;
                    legs[i]->leg_id(i);
                    if(i < 2)
                    {
                        dir = getKneeDirection(gait_config->knee_orientation[0]);
                    }
                    else
                    {
                        dir = getKneeDirection(gait_config->knee_orientation[1]);
                    }
                    legs[i]->is_pantograph(gait_config->pantograph_leg);
                    legs[i]->knee_direction(dir);
                }
            }        
            
            void getJointPositions(float *joint_positions)
            {
                unsigned int total_joints = 0;

                for(unsigned int i = 0; i < 4; i++)
                {
                    joint_positions[total_joints++] = legs[i]->hip->theta();
                    joint_positions[total_joints++] = legs[i]->upper_leg->theta();
                    joint_positions[total_joints++] = legs[i]->lower_leg->theta();
                }
            }

            void getFootPositions(geometry::Transformation *foot_positions)
            {
                for(unsigned int i = 0; i < 4; i++)
                {
                    foot_positions[i] = legs[i]->foot_from_base();
                }
            }

            void updateJointPositions(float joints[12])
            {
                for(unsigned int i = 0; i < 4; i++)
                {
                    int index = i * 3;
                    legs[i]->hip->theta(joints[index]);
                    legs[i]->upper_leg->theta(joints[index + 1]);
                    legs[i]->lower_leg->theta(joints[index + 2]);
                }
            }
            
            // float linear_velocity_x()
            // {
            //     return speed_.linear_velocity_x;
            // }

            // void linear_velocity_x(float linear_velocity_x)
            // {
            //     speed_.linear_velocity_x = linear_velocity_x;
            // }

            // float linear_velocity_y()
            // {
            //     return speed_.linear_velocity_y;
            // }

            // void linear_velocity_y(float linear_velocity_y)
            // {
            //     speed_.linear_velocity_y = linear_velocity_y;
            // }

            // float angular_velocity_z()
            // {
            //     return speed_.angular_velocity_z;
            // }

            // void angular_velocity_z(float angular_velocity_z)
            // {
            //     speed_.angular_velocity_z = angular_velocity_z;
            // }

            // void updateSpeed(Velocities speed)
            // {
            //     speed_.linear_velocity_x = speed.linear_velocity_x;
            //     speed_.linear_velocity_y = speed.linear_velocity_y;
            //     speed_.angular_velocity_z = speed.angular_velocity_z;
            // }

            // void updateSpeed(float linear_velocity_x,  float linear_velocity_y, float angular_velocity_z)
            // {
            //     speed_.linear_velocity_x = linear_velocity_x;
            //     speed_.linear_velocity_y = linear_velocity_y;
            //     speed_.angular_velocity_z = angular_velocity_z;
            // }

            // Velocities speed()
            // {
            //     return speed_;
            // }

            QuadrupedLeg *legs[4];

            QuadrupedLeg *lf;
            QuadrupedLeg *rf;
            QuadrupedLeg *lh;
            QuadrupedLeg *rh;

            GaitConfig *gait_config;
    };
}

#endif