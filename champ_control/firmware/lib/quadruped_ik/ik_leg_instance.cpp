#include<ik_leg_instance.h>

IKLegInstance::IKLegInstance(QuadrupedLeg *leg):
    leg_(leg)
{
}

void IKLegInstance::solve(Transformation target)
{
    Rotation hip_theta;
    joints_[0] = -(atan(target.Z() / target.X()) - ( 1.5708 - acos(leg_->upper_leg->d() / sqrt(pow(target.Z(),2) + pow(target.X(), 2)))));
    target.RotateY(joints_[0]);
    
    // ik for knee forward
    // joints_[2] = acos( (pow(target.X(),2) + pow(target.Y(),2) - pow(leg_->upper_leg->r() ,2) - pow(leg_->lower_leg->r() ,2)) / (2 * leg_->upper_leg->r() * leg_->lower_leg->r()) );
    // joints_[1] = atan(target.Y() / target.X()) - atan( (leg->lower_leg->r() * sin(joints_[2])) / (leg->upper_leg->r() + (leg->lower_leg->r() * cos(joints_[2]))));

    // // reverse
    joints_[2] = -acos((pow(target.X(),2) + pow(target.Y(),2) - pow(leg_->upper_leg->r() ,2) - pow(leg_->lower_leg->r() ,2)) / (2 * leg_->upper_leg->r() * leg_->lower_leg->r()));
    joints_[1] = (atan(target.Y() / target.X()) - atan( (leg_->lower_leg->r() * sin(joints_[2])) / (leg_->upper_leg->r() + (leg_->lower_leg->r() * cos(joints_[2])))));
}

float *IKLegInstance::joints()
{
    return joints_;
}
