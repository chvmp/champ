#include<ik_leg_instance.h>

IKLegInstance::IKLegInstance(QuadrupedLeg *leg):
    leg_(leg)
{
}

void IKLegInstance::solve(Transformation &foot_position, float &hip_joint, float &upper_leg_joint, float &lower_leg_joint)
{
    Rotation hip_theta;
    Point foot_pos = foot_position.p;
    Transformation transformed_foot_position = foot_position;
    
    hip_joint = -(atan(transformed_foot_position.Z() / transformed_foot_position.X()) - (1.5708 - acos(leg_->upper_leg->z() / sqrt(pow(transformed_foot_position.Z(), 2) + pow(transformed_foot_position.X(), 2)))));
    // foot_position.RotateY(joints_[0]);
    hip_theta.RotateY(hip_joint);
    transformed_foot_position.p = hip_theta * foot_pos;
     // // reverse
    lower_leg_joint = -acos((pow(transformed_foot_position.X(), 2) + pow(transformed_foot_position.Y(), 2) - pow(leg_->lower_leg->x() ,2) - pow(leg_->foot->x() ,2)) / (2 * leg_->lower_leg->x() * leg_->foot->x()));
    upper_leg_joint = (atan(transformed_foot_position.Y() / transformed_foot_position.X()) - atan( (leg_->foot->x() * sin(lower_leg_joint)) / (leg_->lower_leg->x() + (leg_->foot->x() * cos(lower_leg_joint)))));

    // if(leg_->leg_id()< 2)
    // {
    //     // // reverse
    //     lower_leg_joint = -acos((pow(transformed_foot_position.X(), 2) + pow(transformed_foot_position.Y(), 2) - pow(leg_->lower_leg->x() ,2) - pow(leg_->foot->x() ,2)) / (2 * leg_->lower_leg->x() * leg_->foot->x()));
    //     upper_leg_joint = (atan(transformed_foot_position.Y() / transformed_foot_position.X()) - atan( (leg_->foot->x() * sin(lower_leg_joint)) / (leg_->lower_leg->x() + (leg_->foot->x() * cos(lower_leg_joint)))));
        
    // }
    // else
    // {
    //     // ik for knee forward
    //     lower_leg_joint = acos((pow(transformed_foot_position.X() , 2) + pow(transformed_foot_position.Y() , 2) - pow(leg_->lower_leg->x() , 2) - pow(leg_->foot->x(), 2)) / (2 * leg_->lower_leg->x() * leg_->foot->x()));
    //     upper_leg_joint = atan(transformed_foot_position.Y() / transformed_foot_position.X()) - atan( (leg_->foot->x() * sin(lower_leg_joint)) / (leg_->lower_leg->x() + (leg_->foot->x() * cos(lower_leg_joint))));
    // }
}

float *IKLegInstance::joints()
{
    return joints_;
}