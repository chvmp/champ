#include<quadruped_gait.h>

QuadrupedGait::QuadrupedGait(QuadrupedBase &quadruped_base, float max_velocity, float swing_height, float step_length, float stance_depth):
    base_(&quadruped_base),
    trajectory_planners_{0,0,0,0},
    phase_gen_(step_length),
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

void QuadrupedGait::generate(Transformation (&foot_positions)[4], float linear_velocity_x, float linear_velocity_y, float angular_velocity_z)
{
    float tangential_velocity = abs(angular_velocity_z * (base_->lf->nominal_stance().Y() +  base_->lf->nominal_stance().X()));
    float velocity =  sqrt(pow(abs(linear_velocity_x) - tangential_velocity, 2) + pow(abs(linear_velocity_y) - tangential_velocity , 2)) ;

    phase_gen_.run(velocity);

    for(unsigned int i = 0; i < 4; i++)
    {
        trajectory_planners_[i]->generate(foot_positions[i], linear_velocity_x, linear_velocity_y, angular_velocity_z, phase_gen_.swing_phase_signal[i], phase_gen_.stance_phase_signal[i]);
    }
}