#ifndef _QUADRUPED_BALANCER_H_
#define _QUADRUPED_BALANCER_H_

#include <Geometry.h>
#include <quadruped_base.h>
#include <balancer_leg_instance.h>

class QuadrupedBalancer
{
    QuadrupedBase *base_;

    BalancerLegInstance *legs_[4];

    public:
        QuadrupedBalancer(QuadrupedBase &quadruped_base);

        void setBodyPose(Transformation (&foot_positions)[4], float target_roll, float target_pitch, 
                        float target_yaw, float target_z);
        
        BalancerLegInstance lf;
        BalancerLegInstance rf;
        BalancerLegInstance lh;
        BalancerLegInstance rh;
};

#endif