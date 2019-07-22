#include<quadruped_gait.h>

QuadrupedGait::QuadrupedGait(QuadrupedBase &quadruped_base, float max_velocity, float swing_height, float step_length, float stance_depth):
    base_(&quadruped_base),
    trajectory_planners_{0,0,0,0},
    phase_gen_(step_length),
    step_length_(step_length),
    lf(base_->lf, swing_height, step_length, stance_depth),
    rf(base_->rf, swing_height, step_length, stance_depth),
    lh(base_->lh, swing_height, step_length, stance_depth),
    rh(base_->rh, swing_height, step_length, stance_depth)
{
    unsigned int total_legs = 0;
    
    trajectory_planners_[total_legs++] = &lf;
    trajectory_planners_[total_legs++] = &rf;
    trajectory_planners_[total_legs++] = &lh;
    trajectory_planners_[total_legs++] = &rh;
}

float QuadrupedGait::getRotation(QuadrupedLeg *leg, float linear_velocity_x, float linear_velocity_y, float angular_velocity_z)
{
    Transformation transformed_stance = leg->nominal_stance();
    transformed_stance.Translate(linear_velocity_x, linear_velocity_y, 0);
    transformed_stance.RotateZ(angular_velocity_z);

    float delta_x = transformed_stance.X() - leg->nominal_stance().X();
    float delta_y = transformed_stance.Y() - leg->nominal_stance().Y();

    return atan2(delta_y, delta_x);
}

void QuadrupedGait::generate(Transformation (&foot_positions)[4], float linear_velocity_x, float linear_velocity_y, float angular_velocity_z)
{
    float tangential_velocity = angular_velocity_z * (base_->lf->nominal_stance().Y() +  base_->lf->nominal_stance().X());
    float velocity =  sqrt(pow(linear_velocity_x, 2) + pow(linear_velocity_y + abs(tangential_velocity), 2)) ;

    phase_gen_.run(abs(velocity));

    for(unsigned int i = 0; i < 4; i++)
    {
        float step_length = 0;
        float rotation = getRotation(base_->legs[i], linear_velocity_x, linear_velocity_y, angular_velocity_z);

        if(!linear_velocity_x && !linear_velocity_y && !angular_velocity_z )  
        {
            step_length = 0;
        }
        else if(linear_velocity_x)
        {
            step_length = step_length_;
        }
        else if(linear_velocity_y)
        {
            step_length = step_length_ * 0.5;
        }
        else if(linear_velocity_x || angular_velocity_z)
        {
            step_length = step_length_ * 0.6;
        }

        trajectory_planners_[i]->generate(foot_positions[i], step_length, rotation, phase_gen_.swing_phase_signal[i], phase_gen_.stance_phase_signal[i]);
    }
}