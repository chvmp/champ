#include<trajectory_planner.h>

TrajectoryPlanner::TrajectoryPlanner(QuadrupedLeg *leg, float max_velocity, float swing_height, float max_foot_displacement):
    leg_(leg),
    max_velocity_(max_velocity),
    max_foot_displacement_(max_foot_displacement),
    stance_depth_(0.0),
    total_control_points_(12),
    gait_started_(false),
    last_cycle_time_(0),
    gait_index_(0),
    factorial_{1.0,1.0,2.0,6.0,24.0,120.0,720.0,5040.0,40320.0,362880.0,3628800.0,39916800.0,479001600.0},
    control_points_x_{-0.15, -0.2805,-0.3,-0.3,-0.3,   0.0, 0.0 ,   0.0, 0.3032, 0.3032, 0.2826, 0.15},
    control_points_y_{0.5,  0.5, 0.3611, 0.3611, 0.3611, 0.3611, 0.3611, 0.3214, 0.3214, 0.3214, 0.5, 0.5}
{
    float height_ratio = swing_height / 0.15;
    float length_ratio = max_foot_displacement_ / 0.4;

    for(unsigned int i = 0; i < 12; i++)
    {
        if(i == 0)
            control_points_x_[i] = -max_foot_displacement_ / 2.0;
        else if(i == 11)
            control_points_x_[i] = max_foot_displacement_ / 2.0;
        else
            control_points_x_[i] = control_points_x_[i] * length_ratio;

        control_points_y_[i] = (control_points_y_[i] * height_ratio) - (0.5 * height_ratio);
    }
}

float TrajectoryPlanner::getGaitCycleCount(float target_velocity)
{
    float temp_cycle_count = 1;

    if(target_velocity == 0)
    {
        return temp_cycle_count;
    }

    target_velocity = constrain(target_velocity, -max_velocity_, max_velocity_);
    temp_cycle_count = target_velocity / max_foot_displacement_;
    return ceil(temp_cycle_count) * 2;
}

void TrajectoryPlanner::generate(Transformation ref, float target_velocity, float rotation, float swing_phase_signal, float stance_phase_signal)
{
    int n = total_control_points_ - 1;
  
    if(swing_phase_signal > 0)
    {
        float x = 0;
        float y = 0;

        for(unsigned int i = 0; i < total_control_points_ ; i++)
        {
            float coeff = factorial_[n] / (factorial_[i] * factorial_[n - i]);

            x += coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_x_[i];
            y += coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_y_[i];
        }

        foot_.X() = ref.X() + y;
        foot_.Y() = ref.Y() - (x * cos(rotation));
        foot_.Z() = ref.Z() + (x * sin(rotation));
    }

    else if(stance_phase_signal > 0)
    {
        float x = (max_foot_displacement_ / 2) * (1 - (2 * stance_phase_signal));
        float y = stance_depth_ * cos((3.1416 * x) / max_foot_displacement_);

        foot_.X() = ref.X() + y;
        foot_.Y() = ref.Y() - (x * cos(rotation));
        foot_.Z() = ref.Z() + (x * sin(rotation));
    }
}
Transformation TrajectoryPlanner::stance()
{
    return foot_;
}