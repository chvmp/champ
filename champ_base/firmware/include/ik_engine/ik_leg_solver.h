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

#ifndef IK_LEG_SOLVER_H
#define IK_LEG_SOLVER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_leg.h>

namespace champ
{
    class IKLegSolver
    {
        QuadrupedLeg *leg_;

        float joints_[3];

        float ik_alpha_;
        float ik_alpha_h_;
        float ik_beta_;
        float ik_beta_h_;

        public:
            IKLegSolver(QuadrupedLeg &leg):
                leg_(&leg),
                ik_alpha_(0.0f),
                ik_alpha_h_(0.0f),
                ik_beta_(0.0f),
                ik_beta_h_(0.0f)
            {
                float upper_to_lower_leg_x = leg_->joint_chain[2]->x();
                float lower_leg_to_foot_x = leg_->joint_chain[3]->x();
                float upper_to_lower_leg_z = leg_->joint_chain[2]->z();
                float lower_leg_to_foot_z = leg_->joint_chain[3]->z();

                ik_alpha_h_ = -sqrtf(pow(upper_to_lower_leg_x, 2) + pow(upper_to_lower_leg_z, 2));
                ik_alpha_ = acosf(upper_to_lower_leg_x / ik_alpha_h_) - (M_PI / 2); 

                ik_beta_h_ = -sqrtf(pow(lower_leg_to_foot_x, 2) + pow(lower_leg_to_foot_z, 2));
                ik_beta_ = acosf(lower_leg_to_foot_x / ik_beta_h_) - (M_PI / 2); 
            }
            
            void solve(float joints[3], geometry::Transformation &foot_position)
            {
                solve(joints[0], joints[1], joints[2], foot_position);
            }

            void solve(float &hip_joint, float &upper_leg_joint, float &lower_leg_joint, geometry::Transformation &foot_position)
            {
                geometry::Transformation temp_foot_pos = foot_position;

                float x = temp_foot_pos.X();
                float y = temp_foot_pos.Z();
                float z = temp_foot_pos.Y();
                float l0 = 0.0f;
                float l1 = ik_alpha_h_;
                float l2 = ik_beta_h_;

                for(unsigned int i = 1; i < 4; i++)
                {
                    l0 += leg_->joint_chain[i]->y();
                }

                hip_joint = -(atanf(z / y) - ((M_PI/2) - acosf(-l0 / sqrtf(pow(z, 2) + pow(y, 2)))));

                temp_foot_pos.RotateX(-hip_joint);
                temp_foot_pos.Translate(-leg_->upper_leg->x(), 0.0f, -leg_->upper_leg->z());

                x = temp_foot_pos.X();
                y = temp_foot_pos.Z();
                z = temp_foot_pos.Y();
                
                lower_leg_joint = leg_->knee_direction() * acosf((pow(y, 2) + pow(x, 2) - pow(l1 ,2) - pow(l2 ,2)) / (2 * l1 * l2));
                upper_leg_joint = (atanf(x / y) - atanf((l2 * sinf(lower_leg_joint)) / (l1 + (l2 * cosf(lower_leg_joint)))));
                
                lower_leg_joint += ik_beta_;
                upper_leg_joint += ik_alpha_;
            }        
    };
}

#endif