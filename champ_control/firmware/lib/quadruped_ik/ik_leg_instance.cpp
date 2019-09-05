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
    float x = transformed_foot_position.X();
    float y = transformed_foot_position.Z();
    float z = transformed_foot_position.Y();
    float l0 = leg_->upper_leg->y();
    float l1 = leg_->lower_leg->z();
    float l2 = leg_->foot->z();

    hip_joint = -(atan(z / y) - (1.5708 - acos(-l0 / sqrt(pow(z, 2) + pow(y, 2)))));
    hip_theta.RotateX(-hip_joint);
    transformed_foot_position.p = hip_theta * foot_pos;
    
    x = transformed_foot_position.X();
    y = transformed_foot_position.Z();
    z = transformed_foot_position.Y();
    
    lower_leg_joint = -acos((pow(y, 2) + pow(x, 2) - pow(l1 ,2) - pow(l2 ,2)) / (2 * l1 * l2));
    upper_leg_joint = (atan(x / y) - atan( (l2 * sin(lower_leg_joint)) / (l1 + (l2 * cos(lower_leg_joint)))));

    // if(leg_->leg_id()< 2)
    // {
    //     // // reverse
    //     lower_leg_joint = -acos((pow(y, 2) + pow(x, 2) - pow(l1 ,2) - pow(l2 ,2)) / (2 * l1 * l2));
    //     upper_leg_joint = (atan(x / y) - atan( (l2 * sin(lower_leg_joint)) / (l1 + (l2 * cos(lower_leg_joint)))));
        
    // }
    // else
    // {
    //     // ik for knee forward
    //     lower_leg_joint = acos((pow(y , 2) + pow(x , 2) - pow(l1 , 2) - pow(l2, 2)) / (2 * l1 * l2));
    //     upper_leg_joint = atan(x / y) - atan( (l2 * sin(lower_leg_joint)) / (l1 + (l2 * cos(lower_leg_joint))));
    // }
}

float *IKLegInstance::joints()
{
    return joints_;
}