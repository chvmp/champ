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

            void poseCommand(geometry::Transformation (&foot_positions)[4], float target_roll, float target_pitch, 
                                    float target_yaw, float target_z)
            {
                for(int i = 0; i < 4; i++)
                {
                    legs_[i]->poseCommand(foot_positions[i], target_roll, target_pitch, target_yaw, target_z);
                }
            }
            
            champ::BalancerLegInstance lf;
            champ::BalancerLegInstance rf;
            champ::BalancerLegInstance lh;
            champ::BalancerLegInstance rh;
    };
}

#endif