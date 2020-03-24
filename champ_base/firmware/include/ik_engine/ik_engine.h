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

#ifndef IK_ENGINE_H
#define IK_ENGINE_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>
#include <ik_engine/ik_leg_solver.h>

namespace champ
{
    class IKEngine
    {
        QuadrupedBase *base;

        IKLegSolver *ik_solvers_[4];

        public:
            IKEngine(QuadrupedBase &quadruped_base):
                base(&quadruped_base),
                lf(*base->lf),
                rf(*base->rf),
                lh(*base->lh),
                rh(*base->rh)
            {
                unsigned int total_legs = 0;

                ik_solvers_[total_legs++] = &lf;
                ik_solvers_[total_legs++] = &rf;
                ik_solvers_[total_legs++] = &lh;
                ik_solvers_[total_legs++] = &rh;
            }

            void solve(float (&joint_positions)[12], geometry::Transformation (&foot_positions)[4])
            {
                float calculated_joints[12];

                for(unsigned int i = 0; i < 4; i++)
                {
                    ik_solvers_[i]->solve(calculated_joints[(i*3)], calculated_joints[(i*3) + 1], calculated_joints[(i*3) + 2], foot_positions[i]);
                    
                    //check if any leg has invalid calculation, if so disregard the whole plan
                    if(isnan(calculated_joints[(i*3)]) || isnan(calculated_joints[(i*3) + 1]) || isnan(calculated_joints[(i*3) + 2]))
                    {
                        return;
                    }
                }
                
                for(unsigned int i = 0; i < 12; i++)
                {
                    joint_positions[i] = calculated_joints[i];
                }
            }

            champ::IKLegSolver lf;
            champ::IKLegSolver rf;
            champ::IKLegSolver lh;
            champ::IKLegSolver rh;
    };
}

#endif