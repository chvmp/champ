#ifndef _GAIT_LEG_INSTANCEH_
#define _GAIT_LEG_INSTANCEH_

#include<Arduino.h>
#include<Geometry.h>

class GaitLegInstance
{
    int frequency_;
    float max_velocity_;
    float max_displacement_;

    bool gait_started_;
    unsigned long  last_cycle_time_;
    unsigned int gait_index_;

    Transformation foot_;

    float getGaitCycleCount(float target_velocity);

    public:
        GaitLegInstance(int frequency, float max_velocity, float max_displacement);
        void generate(Transformation ref, float target_velocity, bool * gait_pattern);
        Transformation stance();
};

#endif