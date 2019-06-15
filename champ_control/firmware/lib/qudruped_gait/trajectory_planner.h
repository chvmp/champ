#ifndef _TRAJECTORY_PLANNER_H_
#define _TRAJECTORY_PLANNER_H_

#include<Arduino.h>
#include<Geometry.h>
#include<quadruped_leg.h>

class TrajectoryPlanner
{
    QuadrupedLeg *leg_;
    float max_velocity_;
    float max_displacement_;

    bool gait_started_;
    unsigned long  last_cycle_time_;
    unsigned int gait_index_;


    float getGaitCycleCount(float target_velocity);
    
    public:
        TrajectoryPlanner(QuadrupedLeg *leg, float max_velocity, float max_displacement);
    
        void generate(Transformation ref, float target_velocity, float swing, float stance);

        Transformation stance();

        Transformation foot_;
};

#endif