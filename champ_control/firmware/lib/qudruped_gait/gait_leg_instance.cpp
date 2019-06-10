#include<gait_leg_instance.h>

GaitLegInstance::GaitLegInstance(QuadrupedLeg *leg, int frequency, float max_velocity, float max_displacement):
    leg_(leg),
    frequency_(frequency),
    max_velocity_(max_velocity),
    max_displacement_(max_displacement),
    gait_started_(false),
    last_cycle_time_(0),
    gait_index_(0)
{
}

float GaitLegInstance::getGaitCycleCount(float target_velocity)
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

void GaitLegInstance::generate(Transformation ref, float target_velocity, bool *gait_pattern)
{
    int cycle_count = getGaitCycleCount(target_velocity);
    float target_displacement = target_velocity / cycle_count;
    float swing_height = target_displacement / PI;
    float a = swing_height / 2;

    if(!gait_started_)
    {
        last_cycle_time_ = millis();
        gait_started_ = true;
    }

    float current_iteration = (millis() - last_cycle_time_) / (float)(1000 / cycle_count);

    if(gait_pattern[gait_index_])
    {
        float theta = current_iteration * (PI * 2);
        foot_.Z() = ref.Z() + (a * (1 - cos(theta)));
        foot_.X() = ref.X() + (a * (theta - sin(theta)));
        foot_.Y() = ref.Y();
    }
    else if(!gait_pattern[gait_index_])
    {
        foot_.Z() = ref.Z();
        foot_.X() = ref.X() + ((1 - current_iteration) * target_displacement);
        foot_.Y() = ref.Y();
    }

    if((millis() - last_cycle_time_) > (float)(1000 / cycle_count))
    {
        last_cycle_time_ = millis();
        gait_index_++;
        if(gait_index_ > 3)
        {
            gait_index_ = 0;
            gait_started_ = false;
        }
    }
}

Transformation GaitLegInstance::stance()
{
    return foot_;
}
