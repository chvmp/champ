
#ifndef _QUADRUPED_IK_H_
#define _QUADRUPED_IK_H_

#include <Geometry.h>
#include <quadruped_base.h>
#include <ik_leg_instance.h>

class QuadrupedIK
{
    QuadrupedBase *base;

    IKLegInstance *ik_solvers_[4];

    public:
        QuadrupedIK(QuadrupedBase &quadruped_base);
        
        void solve(Transformation (&foot_positions)[4], float (&joint_positions)[12]);

        IKLegInstance lf;
        IKLegInstance rf;
        IKLegInstance lh;
        IKLegInstance rh;
};

#endif