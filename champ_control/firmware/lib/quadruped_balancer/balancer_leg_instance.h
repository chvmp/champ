#ifndef _BALANCER_LEG_INSTANCE_
#define _BALANCER_LEG_INSTANCE_

#include<balancer_leg_instance.h>
#include<quadruped_base.h>

class BalancerLegInstance
{
    QuadrupedLeg *leg_;
    
    public:
        BalancerLegInstance(QuadrupedLeg *leg);

        void balance(Transformation &new_foot_position, float body_roll, float body_pitch, 
                                   float body_yaw, float target_z);

        void setBodyPose(Transformation &new_foot_position, float target_roll, float target_pitch, float target_yaw, float target_z);
};

#endif