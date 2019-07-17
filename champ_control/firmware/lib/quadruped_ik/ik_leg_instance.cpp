#include<ik_leg_instance.h>

IKLegInstance::IKLegInstance(QuadrupedLeg *leg):
    leg_(leg)
{
}

void IKLegInstance::solve(Transformation &foot_position, float &hip_joint, float &upper_leg_joint, float &lower_leg_joint)
{
    Rotation hip_theta;
    hip_joint = -(atan(foot_position.Z() / foot_position.X()) - (1.5708 - acos(leg_->upper_leg->d() / sqrt(pow(foot_position.Z(), 2) + pow(foot_position.X(), 2)))));
    // foot_position.RotateY(joints_[0]);
    hip_theta.RotateY(hip_joint);
    foot_position.p = hip_theta * foot_position.p;
     // // reverse
    lower_leg_joint = -acos((pow(foot_position.X(), 2) + pow(foot_position.Y(), 2) - pow(leg_->upper_leg->r() ,2) - pow(leg_->lower_leg->r() ,2)) / (2 * leg_->upper_leg->r() * leg_->lower_leg->r()));
    upper_leg_joint = (atan(foot_position.Y() / foot_position.X()) - atan( (leg_->lower_leg->r() * sin(lower_leg_joint)) / (leg_->upper_leg->r() + (leg_->lower_leg->r() * cos(lower_leg_joint)))));

    // if(leg_->leg_id()< 2)
    // {
    //     // // reverse
    //     lower_leg_joint = -acos((pow(foot_position.X(), 2) + pow(foot_position.Y(), 2) - pow(leg_->upper_leg->r() ,2) - pow(leg_->lower_leg->r() ,2)) / (2 * leg_->upper_leg->r() * leg_->lower_leg->r()));
    //     upper_leg_joint = (atan(foot_position.Y() / foot_position.X()) - atan( (leg_->lower_leg->r() * sin(lower_leg_joint)) / (leg_->upper_leg->r() + (leg_->lower_leg->r() * cos(lower_leg_joint)))));
        
    // }
    // else
    // {
    //     // ik for knee forward
    //     lower_leg_joint = acos((pow(foot_position.X() , 2) + pow(foot_position.Y() , 2) - pow(leg_->upper_leg->r() , 2) - pow(leg_->lower_leg->r(), 2)) / (2 * leg_->upper_leg->r() * leg_->lower_leg->r()));
    //     upper_leg_joint = atan(foot_position.Y() / foot_position.X()) - atan( (leg_->lower_leg->r() * sin(lower_leg_joint)) / (leg_->upper_leg->r() + (leg_->lower_leg->r() * cos(lower_leg_joint))));
    // }
}

float *IKLegInstance::joints()
{
    return joints_;
}