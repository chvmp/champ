#ifndef _TRAJECTORY_PLANNER_H_
#define _TRAJECTORY_PLANNER_H_

#include<Arduino.h>
#include<Geometry.h>
#include<quadruped_leg.h>

class TrajectoryPlanner
{
    QuadrupedLeg *leg_;
    float max_velocity_;
    float max_foot_displacement_;
    float stance_depth_;
    unsigned int total_control_points_;

    bool gait_started_;
    unsigned long  last_cycle_time_;
    unsigned int gait_index_;

    Transformation foot_;

    float factorial_[13];
    float control_points_x_[12];
    float control_points_y_[12];
    
    float getGaitCycleCount(float target_velocity);

    public:
        TrajectoryPlanner(QuadrupedLeg *leg, float max_velocity, float max_foot_displacement);
    
        void generate(Transformation ref, float target_velocity, float swing_phase_signal, float stance_phase_signal);

        Transformation stance();
};

#endif