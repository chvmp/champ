#ifndef _QUADRUPED_BALANCER_H_
#define _QUADRUPED_BALANCER_H_

#include<quadruped_balancer.h>
#include<quadruped_base.h>

class QuadrupedBalancer
{
    Transformation lf_stance_;
    Transformation rf_stance_;
    Transformation lh_stance_;
    Transformation rh_stance_;
    QuadrupedBase * base;
    public:
        QuadrupedBalancer();
        void balance(QuadrupedBase &base, float target_roll, float target_pitch, 
                        float target_yaw, float target_x, float target_y, float target_z);

        void ee_base_to_hip(QuadrupedLeg *leg, Point &point);
        Transformation lf_stance();
        Transformation rf_stance();
        Transformation lh_stance();
        Transformation rh_stance();

};

#endif