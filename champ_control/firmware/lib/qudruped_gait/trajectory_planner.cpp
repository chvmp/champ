#include<trajectory_planner.h>

TrajectoryPlanner::TrajectoryPlanner(QuadrupedLeg *leg, float max_velocity, float max_displacement):
    leg_(leg),
    max_velocity_(max_velocity),
    max_displacement_(max_displacement),
    gait_started_(false),
    last_cycle_time_(0),
    gait_index_(0)
{
}

float TrajectoryPlanner::getGaitCycleCount(float target_velocity)
{
    float temp_cycle_count = 1;

    if(target_velocity == 0)
    {
        return temp_cycle_count;
    }

    target_velocity = constrain(target_velocity, -max_velocity_, max_velocity_);
    temp_cycle_count = target_velocity / max_displacement_;
    return ceil(temp_cycle_count) * 2;
}

void TrajectoryPlanner::generate(Transformation ref, float target_velocity, float swing, float stance)
{
    float ref_y = ref.Y() + (max_displacement_ / 2);
    float swing_height = max_displacement_ / PI;
    float a = swing_height / 2;

    if(swing > 0)
    {
        float theta = swing * (PI * 2);
        foot_.X() = ref.X() - (a * (1 - cos(theta)));
        foot_.Y() = ref_y - (a * (theta - sin(theta)));
        foot_.Z() = ref.Z();
    }
    else if(stance > 0)
    {
        foot_.X() = ref.X();
        foot_.Y() = ref_y - ((1 - stance) * max_displacement_);
        foot_.Z() = ref.Z();
    }
}

Transformation TrajectoryPlanner::stance()
{
    return foot_;
}