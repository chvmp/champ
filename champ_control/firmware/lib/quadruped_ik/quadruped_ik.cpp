#include<quadruped_ik.h>

QuadrupedIK::QuadrupedIK(QuadrupedBase &quadruped_base)
{
    base = &quadruped_base;
}

void QuadrupedIK::solveLeg(QuadrupedLeg *leg, Transformation target, float *joints)
{
    Rotation hip_theta;
    joints[0] = -(atan(target.Z() / target.X()) - ( 1.5708 - acos(leg->upper_leg->d() / sqrt(pow(target.Z(),2) + pow(target.X(), 2)))));
    target.RotateY(joints[0]);
    
    // // ik for knee forward
    // // joints[2] = acos( (pow(target.X(),2) + pow(target.Y(),2) - pow(leg->upper_leg->r() ,2) - pow(leg->lower_leg->r() ,2)) / (2 * leg->upper_leg->r() * leg->lower_leg->r()) );
    // // joints[1] = atan(target.Y() / target.X()) - atan( (leg->lower_leg->r() * sin(joints[2])) / (leg->upper_leg->r() + (leg->lower_leg->r() * cos(joints[2]))));

    // // reverse
    joints[2] = -acos((pow(target.X(),2) + pow(target.Y(),2) - pow(leg->upper_leg->r() ,2) - pow(leg->lower_leg->r() ,2)) / (2 * leg->upper_leg->r() * leg->lower_leg->r()));
    joints[1] = (atan(target.Y() / target.X()) - atan( (leg->lower_leg->r() * sin(joints[2])) / (leg->upper_leg->r() + (leg->lower_leg->r() * cos(joints[2])))));
}

void QuadrupedIK::solveBody(Transformation lf_target, Transformation rf_target, Transformation lh_target, Transformation rh_target)
{
    solveLeg(base->legs[0],lf_target, lf_joints_);
    solveLeg(base->legs[1],rf_target, rf_joints_);
    solveLeg(base->legs[2],lh_target, lh_joints_);
    solveLeg(base->legs[3],rh_target, rh_joints_);
}

float *QuadrupedIK::lf_joints()
{
    return lf_joints_;
}

float *QuadrupedIK::rf_joints()
{
    return rf_joints_;
}

float *QuadrupedIK::lh_joints()
{
    return lh_joints_;
}

float *QuadrupedIK::rh_joints()
{
    return rh_joints_;
}