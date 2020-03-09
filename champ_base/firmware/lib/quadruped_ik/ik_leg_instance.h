#ifndef _IK_LEG_INSTNACE_H_
#define _IK_LEG_INSTNACE_H_

#include <Geometry.h>
#include <quadruped_leg.h>

class IKLegInstance
{
    QuadrupedLeg *leg_;

    float joints_[3];

    float ik_alpha_;
    float ik_alpha_h_;
    float ik_beta_;
    float ik_beta_h_;

    public:
        IKLegInstance(QuadrupedLeg *leg);
        
        void solve(Transformation &foot_position, float &hip_joint, float &upper_leg_joint, float &lower_leg_joint);
        float *joints();
};

#endif