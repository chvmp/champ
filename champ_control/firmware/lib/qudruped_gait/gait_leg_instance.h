#ifndef _GAIT_LEG_INSTANCEH_
#define _GAIT_LEG_INSTANCEH_

#include<Arduino.h>
#include<Geometry.h>
#include<balancer_leg_instance.h>

class GaitLegInstance
{
    QuadrupedLeg *leg_;
    int frequency_;
    float max_velocity_;
    float max_displacement_;

    bool gait_started_;
    unsigned long  last_cycle_time_;
    unsigned int gait_index_;


    float getGaitCycleCount(float target_velocity);
    
    public:
        Transformation foot_;

        GaitLegInstance(QuadrupedLeg *leg, int frequency, float max_velocity, float max_displacement);
        void generate(Transformation ref, float target_velocity, bool * gait_pattern);

        Transformation stance();
};

#endif