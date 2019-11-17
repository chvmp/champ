#include<quadruped_gait.h>

QuadrupedGait::QuadrupedGait(QuadrupedBase &quadruped_base, float max_linear_velocity_x,float max_linear_velocity_y, float max_angular_velocity, float max_step_length, float max_theta, float swing_height, float stance_depth):
    base_(&quadruped_base),
    trajectory_planners_{0,0,0,0},
    phase_gen_(),
    max_linear_velocity_x_(max_linear_velocity_x),
    max_linear_velocity_y_(max_linear_velocity_y),
    max_angular_velocity_(max_angular_velocity),
    max_step_length_(max_step_length),
    max_theta_(max_theta),
    lf(base_->lf, swing_height, max_step_length_, stance_depth),
    rf(base_->rf, swing_height, max_step_length_, stance_depth),
    lh(base_->lh, swing_height, max_step_length_, stance_depth),
    rh(base_->rh, swing_height, max_step_length_, stance_depth)
{
    unsigned int total_legs = 0;
    
    trajectory_planners_[total_legs++] = &lf;
    trajectory_planners_[total_legs++] = &rf;
    trajectory_planners_[total_legs++] = &lh;
    trajectory_planners_[total_legs++] = &rh;
}

void QuadrupedGait::transformTrajectory(QuadrupedLeg *leg, float linear_velocity_x, float linear_velocity_y, float angular_velocity_z, float &step_length, float &rotation)
{    
    float theta = 0;

    linear_velocity_x = constrain(linear_velocity_x, -max_linear_velocity_x_, max_linear_velocity_x_);
    linear_velocity_y = constrain(linear_velocity_y, -max_linear_velocity_y_, max_linear_velocity_y_);
    angular_velocity_z = constrain(angular_velocity_z, -max_angular_velocity_, max_angular_velocity_);

    float step_x = mapFloat(linear_velocity_x, -max_linear_velocity_x_, max_linear_velocity_x_, -max_step_length_, max_step_length_);
    float step_y = mapFloat(linear_velocity_y, -max_linear_velocity_y_, max_linear_velocity_y_, -max_step_length_, max_step_length_);
    float step_theta = mapFloat(angular_velocity_z, -max_angular_velocity_, max_angular_velocity_, -max_theta_, max_theta_);

    if(angular_velocity_z)
    {
        theta = step_theta;
    }

    Transformation transformed_stance = leg->nominal_stance();
    
    transformed_stance.RotateZ(theta);
    transformed_stance.Translate(step_x, step_y, 0);

    float delta_x = transformed_stance.X() - leg->nominal_stance().X();
    float delta_y = transformed_stance.Y() - leg->nominal_stance().Y();

    rotation = atan2f(delta_y, delta_x);
    step_length = sqrtf(pow(delta_x, 2) + pow(delta_y, 2));
}

void QuadrupedGait::generate(Transformation (&foot_positions)[4], float linear_velocity_x, float linear_velocity_y, float angular_velocity_z)
{
    float tangential_velocity = angular_velocity_z * (base_->lf->nominal_stance().Y() +  base_->lf->nominal_stance().X());
    float velocity =  sqrtf(pow(linear_velocity_x, 2) + pow(linear_velocity_y + abs(tangential_velocity), 2));
    float step_lengths[4] = {0,0,0,0};
    float trajectory_rotations[4] = {0,0,0,0};    
    float sum_of_steps = 0;

    for(unsigned int i = 0; i < 4; i++)
    {
        transformTrajectory(base_->legs[i], linear_velocity_x, linear_velocity_y, angular_velocity_z, step_lengths[i], trajectory_rotations[i]);
        sum_of_steps += step_lengths[i];
    }

    phase_gen_.run(velocity, sum_of_steps / 4);

    for(unsigned int i = 0; i < 4; i++)
    {
        trajectory_planners_[i]->generate(foot_positions[i], step_lengths[i], trajectory_rotations[i], phase_gen_.swing_phase_signal[i], phase_gen_.stance_phase_signal[i]);
    }
}

float QuadrupedGait::mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}