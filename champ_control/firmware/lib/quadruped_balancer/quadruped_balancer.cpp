#include<quadruped_balancer.h>
#include <BasicLinearAlgebra.h>

QuadrupedBalancer::QuadrupedBalancer(QuadrupedBase &quadruped_base):
    base(&quadruped_base),
    lf(base->lf),
    rf(base->rf),
    lh(base->lh),
    rh(base->rh)
{
    unsigned int total_stances = 0;

    legs_[total_stances++] = &lf;
    legs_[total_stances++] = &rf;
    legs_[total_stances++] = &lh;
    legs_[total_stances++] = &rh;
}

void QuadrupedBalancer::balance(float target_roll, float target_pitch, 
                        float target_yaw, float target_x, float target_y, float target_z)
{
    for(int i = 0; i < 4; i++)
    {
        legs_[i]->legGroundIntersection(target_roll, target_pitch, target_yaw, target_x, target_y, target_z);
    }
}
