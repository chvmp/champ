#ifndef _QUADRUPED_IK_H_
#define _QUADRUPED_IK_H_

#include<quadruped_base.h>
#include<Geometry.h>

class QuadrupedIK
{
    QuadrupedBase *base;
    public:
        QuadrupedIK();
      
        void solveLeg(QuadrupedLeg *leg, Point target, float *joints);
        void solveBody(QuadrupedBase &base, Point lf_target, Point rf_target, Point lh_target, Point rh_target, float *joints);
};

#endif