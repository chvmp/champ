
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

            void solve(float (&joint_positions)[12], Transformation (&foot_positions)[4])
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