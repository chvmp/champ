#ifndef _BALANCER_LEG_INSTANCE_
#define _BALANCER_LEG_INSTANCE_

#include<balancer_leg_instance.h>
#include<quadruped_base.h>

class BalancerLegInstance
{
    Transformation foot_;
    QuadrupedLeg *leg_;

    public:
        BalancerLegInstance(QuadrupedLeg *leg);

        void balance(float target_roll, float target_pitch, float target_yaw, 
                     float target_x, float target_y, float target_z);

        void legGroundIntersection(float target_roll, float target_pitch, 
                                   float target_yaw, float target_x, float target_y, float target_z);

        void ee_base_to_hip(Transformation &foot);

        Transformation stance();
};

#endif