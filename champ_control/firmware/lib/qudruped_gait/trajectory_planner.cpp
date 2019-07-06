#include<trajectory_planner.h>

TrajectoryPlanner::TrajectoryPlanner(float swing_height, float stance_depth):
    swing_height_(swing_height),
    stance_depth_(stance_depth),
    total_control_points_(12),
    gait_started_(false),
    last_cycle_time_(0),
    gait_index_(0),
    factorial_{1.0,1.0,2.0,6.0,24.0,120.0,720.0,5040.0,40320.0,362880.0,3628800.0,39916800.0,479001600.0},
    ref_control_points_x_{-0.15, -0.2805,-0.3,-0.3,-0.3,   0.0, 0.0 ,   0.0, 0.3032, 0.3032, 0.2826, 0.15},
    ref_control_points_y_{0.5,  0.5, 0.3611, 0.3611, 0.3611, 0.3611, 0.3611, 0.3214, 0.3214, 0.3214, 0.5, 0.5},
    control_points_x_{-0.15, -0.2805,-0.3,-0.3,-0.3,   0.0, 0.0 ,   0.0, 0.3032, 0.3032, 0.2826, 0.15},
    control_points_y_{0.5,  0.5, 0.3611, 0.3611, 0.3611, 0.3611, 0.3611, 0.3214, 0.3214, 0.3214, 0.5, 0.5},
    height_ratio_(0),
    length_ratio_(0)
{
    updateControlPointsHeight(swing_height_);
}

void TrajectoryPlanner::updateControlPointsHeight(float swing_height)
{
    float new_height_ratio = swing_height / 0.15;
    
    if(height_ratio_ != new_height_ratio)
    {
        height_ratio_ = new_height_ratio;
        for(unsigned int i = 0; i < 12; i++)
        {
            control_points_y_[i] = (ref_control_points_y_[i] * height_ratio_) - (0.5 * height_ratio_);
        }    
    }
}

void TrajectoryPlanner::updateControlPointsLength(float step_length)
{
    float new_length_ratio = step_length / 0.4;
    
    if(length_ratio_ != new_length_ratio)
    {
        length_ratio_ = new_length_ratio;
        for(unsigned int i = 0; i < 12; i++)
        {
            if(i == 0)
                control_points_x_[i] = -step_length / 2.0;
            else if(i == 11)
                control_points_x_[i] = step_length / 2.0;
            else
                control_points_x_[i] = ref_control_points_x_[i] * length_ratio_;   
        }
    }
}

void TrajectoryPlanner::generate(Transformation ref, float rotation, float step_distance, float swing_phase_signal, float stance_phase_signal)
{
    updateControlPointsLength(step_distance);
    
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
        float x = (step_distance / 2) * (1 - (2 * stance_phase_signal));
        float y = stance_depth_ * cos((3.1416 * x) / step_distance);

        foot_.X() = ref.X() + y;
        foot_.Y() = ref.Y() - (x * cos(rotation));
        foot_.Z() = ref.Z() + (x * sin(rotation));
    }
}

Transformation TrajectoryPlanner::stance()
{
    return foot_;
}