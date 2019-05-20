#ifndef _QUADRUPED_IK_H_
#define _QUADRUPED_IK_H_

#include<quadruped_base.h>

class QuadrupedIK
{
    QuadrupedBase *base;
    public:
        QuadrupedIK();
      
        void solveLeg(const QuadrupedLeg *leg, Transformation &lf_target, float *joints);
        void solveBody(const QuadrupedBase &base, Transformation &lf_target, Transformation &rf_target,
                                             Transformation &lh_target, Transformation &rh_target, float *joints);
};
#endif