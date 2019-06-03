#ifndef _QUADRUPED_IK_H_
#define _QUADRUPED_IK_H_

#include<quadruped_base.h>
#include<ik_leg_instance.h>

#include<Geometry.h>

class QuadrupedIK
{
    QuadrupedBase *base;

    IKLegInstance *legs_[4];

    public:
        IKLegInstance lf;
        IKLegInstance rf;
        IKLegInstance lh;
        IKLegInstance rh;
        
        QuadrupedIK(QuadrupedBase &quadruped_base);
        void solveBody(Transformation lf_target, Transformation rf_target, Transformation lh_target, Transformation rh_target);
};

#endif