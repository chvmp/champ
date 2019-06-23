#include<trajectory_planner.h>

TrajectoryPlanner::TrajectoryPlanner(QuadrupedLeg *leg, float max_velocity, float max_foot_displacement):
    leg_(leg),
    max_velocity_(max_velocity),
    max_foot_displacement_(max_foot_displacement),
    stance_depth_(0),
    total_control_points_(12),
    gait_started_(false),
    last_cycle_time_(0),
    gait_index_(0),
    factorial_{1.0,1.0,2.0,6.0,24.0,120.0,720.0,5040.0,40320.0,362880.0,3628800.0,39916800.0,479001600.0},
    control_points_x_{-0.0625, -0.08415, -0.09, -0.09, -0.09, 0.0, 0.0, 0.0, 0.09096, 0.09096, 0.08478000000000001, 0.0625},
    control_points_y_{0.0, 0.0, -0.04167, -0.04167, -0.04167, -0.04167, -0.04167, -0.05357999999999999, -0.05357999999999999, -0.05357999999999999, 0.0, 0.0}
{
    control_points_x_[0] = -max_foot_displacement_ / 2;    
    control_points_x_[total_control_points_ - 1] = max_foot_displacement_ / 2;
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

void TrajectoryPlanner::generate(Transformation ref, float target_velocity, float swing_phase_signal, float stance_phase_signal)
{
    int n = total_control_points_ - 1;
    float x = 0;
    float y = 0;

    if(swing_phase_signal > 0)
    {
        for(unsigned int i = 0; i < total_control_points_ ; i++)
        {
            float coeff = factorial_[n] / (factorial_[i] * factorial_[n - i]);

            x += coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_x_[i];
            y += coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_y_[i];
        }

        foot_.X() = ref.X() + y;
        foot_.Y() = ref.Y() - x;
        foot_.Z() = ref.Z();
    }
    else if(stance_phase_signal > 0)
    {
        foot_.X() = ref.X() + stance_depth_ * cos((3.1416 * x) / max_foot_displacement_);
        foot_.Y() = ref.Y() - (max_foot_displacement_ / 2) * (1 - (2 * stance_phase_signal));
        foot_.Z() = ref.Z();
    }
}

Transformation TrajectoryPlanner::stance()
{
    return foot_;
}