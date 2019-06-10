#include<quadruped_ik.h>

QuadrupedIK::QuadrupedIK(QuadrupedBase &quadruped_base):
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

void QuadrupedIK::solve(Transformation lf_target, Transformation rf_target, Transformation lh_target, Transformation rh_target)
{
    lf.solve(lf_target);
    rf.solve(rf_target);
    lh.solve(lh_target);
    rh.solve(rh_target);
}
