#ifndef _QUADRUPED_BALANCER_H_
#define _QUADRUPED_BALANCER_H_

#include<balancer_leg_instance.h>
#include<quadruped_balancer.h>
#include<quadruped_base.h>

class QuadrupedBalancer
{
    QuadrupedBase *base;

    BalancerLegInstance *legs_[4];
    
    // Transformation lf_stance_;
    // Transformation rf_stance_;
    // Transformation lh_stance_;
    // Transformation rh_stance_;

    public:
        BalancerLegInstance lf;
        BalancerLegInstance rf;
        BalancerLegInstance lh;
        BalancerLegInstance rh;

        QuadrupedBalancer(QuadrupedBase &quadruped_base);

        void balance(float target_roll, float target_pitch, float target_yaw, 
                     float target_x, float target_y, float target_z);
};

#endif