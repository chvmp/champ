#include<ik_leg_instance.h>

IKLegInstance::IKLegInstance(QuadrupedLeg *leg):
    leg_(leg),
    ik_alpha_(0),
    ik_beta_(0),
    ik_gamma_(0)
{
    float hip_to_upper_leg_y = 0;
    float upper_to_lower_leg_x = leg_->joint_chain[2]->x();
    float lower_leg_to_foot_x =  leg_->joint_chain[3]->x();
    float upper_leg_length = leg_->joint_chain[2]->z();
    float lower_leg_length = leg_->joint_chain[3]->z();

    for(unsigned int i = 1; i < 4; i++)
    {
        hip_to_upper_leg_y += leg_->joint_chain[i]->y();
    }

    float alpha_h = sqrt(pow(hip_to_upper_leg_y, 2) + pow(leg_->nominal_stance().Z(),2));
    float alpha_phi = acos(hip_to_upper_leg_y / alpha_h); 
    ik_alpha_ = (PI / 2) + alpha_phi;

    float beta_h = sqrt(pow(upper_to_lower_leg_x, 2) + pow(upper_leg_length, 2));
    float beta_phi = asin(upper_to_lower_leg_x / beta_h); 
    ik_beta_ = beta_phi;

    float gamma_h = sqrt(pow(lower_leg_to_foot_x, 2) + pow(lower_leg_length, 2));
    float gamma_phi = asin(lower_leg_to_foot_x / gamma_h); 
    ik_gamma_ = gamma_phi;
}

void IKLegInstance::solve(Transformation &foot_position, float &hip_joint, float &upper_leg_joint, float &lower_leg_joint)
{
    Rotation hip_theta;
    Point foot_pos = foot_position.p;
    Transformation transformed_foot_position = foot_position;
    float x = transformed_foot_position.X();
    float y = transformed_foot_position.Z();
    float z = transformed_foot_position.Y();
    float l0 = 0;
    float l1 = leg_->lower_leg->z();
    float l2 = leg_->foot->z();

    for(unsigned int i = 1; i < 4; i++)
    {
        l0 += leg_->joint_chain[i]->y();
    }

    hip_joint = -(atan(z / y) - (1.5708 - acos(-l0 / sqrt(pow(z, 2) + pow(y, 2)))));
    // hip_joint = -atan2(z, y);
    // hip_joint -= ik_alpha_;

    hip_theta.RotateX(-hip_joint);
    transformed_foot_position.p = hip_theta * foot_pos;

    x = transformed_foot_position.X();
    y = transformed_foot_position.Z();
    z = transformed_foot_position.Y();
    
    lower_leg_joint = leg_->knee_direction() * acos((pow(y, 2) + pow(x, 2) - pow(l1 ,2) - pow(l2 ,2)) / (2 * l1 * l2));
    lower_leg_joint += ik_gamma_;

    upper_leg_joint = (atan(x / y) - atan( (l2 * sin(lower_leg_joint)) / (l1 + (l2 * cos(lower_leg_joint)))));
    upper_leg_joint += ik_beta_ + ik_gamma_;
}        



float *IKLegInstance::joints()
{
    return joints_;
}