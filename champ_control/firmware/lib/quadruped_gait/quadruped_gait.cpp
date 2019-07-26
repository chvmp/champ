#include<quadruped_gait.h>

QuadrupedGait::QuadrupedGait(QuadrupedBase &quadruped_base, float max_velocity, float swing_height, float step_length, float stance_depth):
    base_(&quadruped_base),
    trajectory_planners_{0,0,0,0},
    phase_gen_(),
    step_length_(step_length),
    transversal_step_length_(0),
    lateral_step_length_(0),
    rotational_step_length_(0),
    optimal_rotational_angle_(0),
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

    transversal_step_length_ = step_length_;
    lateral_step_length_ = step_length_ * 0.5;
    rotational_step_length_ = step_length_ * 0.4;

    optimal_rotational_angle_ = getOptimalRotationalAngle(base_->lf, rotational_step_length_);
}

void QuadrupedGait::transformTrajectory(QuadrupedLeg *leg, float linear_velocity_x, float linear_velocity_y, float angular_velocity_z, float &step_length, float &rotation)
{    
    float turning_radius = 0;
    float theta = 0;

    //calculate the angle of rotation from linear and angular velocity
    //to transform the base
    if(linear_velocity_x && angular_velocity_z)
    {
        turning_radius = linear_velocity_x / angular_velocity_z;
        theta = atan((base_->lf->x() * 2) / turning_radius);
    }
    else if(angular_velocity_z)
    {
        theta = (angular_velocity_z * optimal_rotational_angle_) / abs(angular_velocity_z);
    }

    Transformation transformed_stance = leg->nominal_stance();
    
    transformed_stance.RotateZ(theta);
    transformed_stance.Translate(linear_velocity_x, linear_velocity_y, 0);

    float delta_x = transformed_stance.X() - leg->nominal_stance().X();
    float delta_y = transformed_stance.Y() - leg->nominal_stance().Y();

    rotation = atan2(delta_y, delta_x);

    float vx = 0;
    float vy = 0;

    if(linear_velocity_x)
        vx = (delta_x * transversal_step_length_) / abs(delta_x);
    
    if(linear_velocity_y)
        vy = (delta_y * lateral_step_length_) / abs(delta_y);

    if(!linear_velocity_x && !linear_velocity_y && angular_velocity_z)
        step_length = rotational_step_length_;

    else if(linear_velocity_y && (linear_velocity_x && linear_velocity_y))
        step_length = lateral_step_length_;

    else
        step_length = sqrt(pow(vx, 2) + pow(vy, 2));
}

float QuadrupedGait::getOptimalRotationalAngle(QuadrupedLeg *leg, float step_length)
{
    for(unsigned int i = 0; i < 100; i++)
    {
        float theta = (float)(i * 0.0174533);
        Transformation transformed_stance = leg->nominal_stance();
        transformed_stance.RotateZ(theta);

        Point d = transformed_stance.p - leg->nominal_stance().p;

        float error = step_length - d.Magnitude();

        if(error < 0.002 && error > 0)
        {
            rotational_step_length_ = abs(d.Magnitude());
            return abs(theta);
        }
    }

    return 0;
}

void QuadrupedGait::generate(Transformation (&foot_positions)[4], float linear_velocity_x, float linear_velocity_y, float angular_velocity_z)
{
    float tangential_velocity = angular_velocity_z * (base_->lf->nominal_stance().Y() +  base_->lf->nominal_stance().X());
    float velocity =  sqrt(pow(linear_velocity_x, 2) + pow(linear_velocity_y + abs(tangential_velocity), 2));
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