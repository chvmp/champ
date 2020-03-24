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

#ifndef BODY_CONTROLLER_H
#define BODY_CONTROLLER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>
#include <body_controller/balancer_leg_instance.h>

namespace champ
{
    class BodyController
    {
        QuadrupedBase *base_;

        champ::BalancerLegInstance *legs_[4];

        public:
            BodyController(QuadrupedBase &quadruped_base):
                base_(&quadruped_base),
                lf(*base_->lf),
                rf(*base_->rf),
                lh(*base_->lh),
                rh(*base_->rh)
            {
                unsigned int total_stances = 0;

                legs_[total_stances++] = &lf;
                legs_[total_stances++] = &rf;
                legs_[total_stances++] = &lh;
                legs_[total_stances++] = &rh;
            }

            void poseCommand(geometry::Transformation (&foot_positions)[4], champ::Pose &req_pose)
            {
                for(int i = 0; i < 4; i++)
                {
                    legs_[i]->poseCommand(foot_positions[i], req_pose);
                }
            }
            
            champ::BalancerLegInstance lf;
            champ::BalancerLegInstance rf;
            champ::BalancerLegInstance lh;
            champ::BalancerLegInstance rh;
    };
}

#endif