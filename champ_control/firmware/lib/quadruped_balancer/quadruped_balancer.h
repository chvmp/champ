#ifndef _QUADRUPED_BALANCER_H_
#define _QUADRUPED_BALANCER_H_

#include<balancer_leg_instance.h>
#include<quadruped_balancer.h>
#include<quadruped_base.h>

class QuadrupedBalancer
{
    QuadrupedBase *base_;

    BalancerLegInstance *legs_[4];

    public:
        QuadrupedBalancer(QuadrupedBase &quadruped_base);

        void balance(Transformation (&foot_positions)[4], float target_roll, float target_pitch, 
                        float target_yaw, float target_x, float target_y, float target_z);
        
        BalancerLegInstance lf;
        BalancerLegInstance rf;
        BalancerLegInstance lh;
        BalancerLegInstance rh;
};

#endif