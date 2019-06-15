#ifndef _IK_LEG_INSTNACE_H_
#define _IK_LEG_INSTNACE_H_

#include<quadruped_base.h>
#include<Geometry.h>

class IKLegInstance
{
    QuadrupedLeg *leg_;

    float joints_[3];

    public:
        IKLegInstance(QuadrupedLeg *leg);
        
        void solve(Transformation target);
        float *joints();
};

#endif