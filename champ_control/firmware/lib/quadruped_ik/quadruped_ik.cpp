#include<quadruped_ik.h>

QuadrupedIK::QuadrupedIK(QuadrupedBase &quadruped_base):
    base(&quadruped_base),
    lf(base->lf),
    rf(base->rf),
    lh(base->lh),
    rh(base->rh)
{
    unsigned int total_legs = 0;

    ik_solvers_[total_legs++] = &lf;
    ik_solvers_[total_legs++] = &rf;
    ik_solvers_[total_legs++] = &lh;
    ik_solvers_[total_legs++] = &rh;
}

void QuadrupedIK::solve(Transformation (&foot_positions)[4], float (&joint_positions)[12])
{
    for(unsigned int i = 0; i < 4; i++)
    {
        ik_solvers_[i]->solve(foot_positions[i], joint_positions[(i*3)], joint_positions[(i*3) + 1], joint_positions[(i*3) + 2]);
    }
}