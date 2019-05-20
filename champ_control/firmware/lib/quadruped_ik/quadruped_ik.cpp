#include<quadruped_ik.h>

QuadrupedIK::QuadrupedIK()
{
}

void QuadrupedIK::solveLeg(QuadrupedLeg *leg, Point target, float *joints)
{
    Rotation hip_theta;
    joints[0] = -(atan(target.Z() / target.X()) - ( 1.5708 - acos(leg->upper_leg->d() / sqrt(pow(target.Z(),2) + pow(target.X(), 2)))));
    hip_theta.RotateY(joints[0]);
    target = hip_theta * target;
    
    // // ik for knee forward
    // // joints[2] = acos( (pow(target.X(),2) + pow(target.Y(),2) - pow(leg->upper_leg->r() ,2) - pow(leg->lower_leg->r() ,2)) / (2 * leg->upper_leg->r() * leg->lower_leg->r()) );
    // // joints[1] = atan(target.Y() / target.X()) - atan( (leg->lower_leg->r() * sin(joints[2])) / (leg->upper_leg->r() + (leg->lower_leg->r() * cos(joints[2]))));

    // // reverse
    joints[2] = -acos((pow(target.X(),2) + pow(target.Y(),2) - pow(leg->upper_leg->r() ,2) - pow(leg->lower_leg->r() ,2)) / (2 * leg->upper_leg->r() * leg->lower_leg->r()));
    joints[1] = (atan(target.Y() / target.X()) - atan( (leg->lower_leg->r() * sin(joints[2])) / (leg->upper_leg->r() + (leg->lower_leg->r() * cos(joints[2])))));
  
}

void QuadrupedIK::solveBody(QuadrupedBase &base, Point lf_target, Point rf_target, Point lh_target, Point rh_target, float *joints)
{
    unsigned int total_joints = 0;
    Point *legs[4];

    legs[0] = &lf_target;
    legs[1] = &rf_target;
    legs[2] = &lh_target;
    legs[3] = &rh_target;

    for(unsigned int i = 0; i < 4; i++)
    {
        float temp_joints[3];
        solveLeg(base.legs[i], *legs[i], temp_joints);
        joints[total_joints++] = temp_joints[0];
        joints[total_joints++] = temp_joints[1];
        joints[total_joints++] = temp_joints[2];
    }
}