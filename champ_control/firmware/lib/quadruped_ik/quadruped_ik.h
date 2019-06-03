#ifndef _QUADRUPED_IK_H_
#define _QUADRUPED_IK_H_

#include<quadruped_base.h>
#include<Geometry.h>

class QuadrupedIK
{
    QuadrupedBase *base;

    float lf_joints_[3];
    float rf_joints_[3];
    float lh_joints_[3];
    float rh_joints_[3];

    public:
        QuadrupedIK(QuadrupedBase &quadruped_base);
      
        void solveLeg(QuadrupedLeg *leg, Transformation target, float *joints);
        void solveBody(Transformation lf_target, Transformation rf_target, Transformation lh_target, Transformation rh_target);
        
        float *lf_joints();
        float *rf_joints();
        float *lh_joints();
        float *rh_joints();
};

#endif