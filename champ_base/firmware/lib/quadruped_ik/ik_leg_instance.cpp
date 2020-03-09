#include <ik_leg_instance.h>

IKLegInstance::IKLegInstance(QuadrupedLeg *leg):
    leg_(leg),
    ik_alpha_(0),
    ik_alpha_h_(0),
    ik_beta_(0),
    ik_beta_h_(0)
{
    float upper_to_lower_leg_x = leg_->joint_chain[2]->x();
    float lower_leg_to_foot_x = leg_->joint_chain[3]->x();
    float upper_to_lower_leg_z = leg_->joint_chain[2]->z();
    float lower_leg_to_foot_z = leg_->joint_chain[3]->z();

    ik_alpha_h_ = -sqrtf(pow(upper_to_lower_leg_x, 2) + pow(upper_to_lower_leg_z, 2));
    ik_alpha_ = acosf(upper_to_lower_leg_x / ik_alpha_h_) - (PI/2); 

    ik_beta_h_ = -sqrtf(pow(lower_leg_to_foot_x, 2) + pow(lower_leg_to_foot_z, 2));
    ik_beta_ = acosf(lower_leg_to_foot_x / ik_beta_h_) - (PI/2); 
}

void IKLegInstance::solve(Transformation &foot_position, float &hip_joint, float &upper_leg_joint, float &lower_leg_joint)
{
    Transformation temp_foot_pos = foot_position;

    float x = temp_foot_pos.X();
    float y = temp_foot_pos.Z();
    float z = temp_foot_pos.Y();
    float l0 = 0;
    float l1 = ik_alpha_h_;
    float l2 = ik_beta_h_;

    for(unsigned int i = 1; i < 4; i++)
    {
        l0 += leg_->joint_chain[i]->y();
    }

    hip_joint = -(atanf(z / y) - ((PI/2) - acosf(-l0 / sqrtf(pow(z, 2) + pow(y, 2)))));

    temp_foot_pos.RotateX(-hip_joint);
    temp_foot_pos.Translate(-leg_->upper_leg->x(), 0, -leg_->upper_leg->z());

    x = temp_foot_pos.X();
    y = temp_foot_pos.Z();
    z = temp_foot_pos.Y();
    
    lower_leg_joint = leg_->knee_direction() * acosf((pow(y, 2) + pow(x, 2) - pow(l1 ,2) - pow(l2 ,2)) / (2 * l1 * l2));
    upper_leg_joint = (atanf(x / y) - atanf((l2 * sinf(lower_leg_joint)) / (l1 + (l2 * cosf(lower_leg_joint)))));
    
    lower_leg_joint += ik_beta_;
    upper_leg_joint += ik_alpha_;
}        

float *IKLegInstance::joints()
{
    return joints_;
}