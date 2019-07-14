#ifndef _TRAJECTORY_PLANNER_H_
#define _TRAJECTORY_PLANNER_H_

#include<Arduino.h>
#include<Geometry.h>
#include<quadruped_leg.h>

class TrajectoryPlanner
{
    QuadrupedLeg *leg_;

    float swing_height_;
    float step_length_;
    float stance_depth_;
    unsigned int total_control_points_;

    Transformation foot_;

    float factorial_[13];
    float ref_control_points_x_[12];
    float ref_control_points_y_[12];
    float control_points_x_[12];
    float control_points_y_[12];
    
    float height_ratio_;
    float length_ratio_;

    float getGaitCycleCount(float target_velocity);
    void updateControlPointsHeight(float swing_height);
    void updateControlPointsLength(float step_length);

    public:
        TrajectoryPlanner(QuadrupedLeg *leg, float swing_height, float step_length, float stance_depth);
        void generate(Transformation ref, float linear_vel_x, float linear_vel_y, float angular_vel_z, float swing_phase_signal, float stance_phase_signal);
        
        Transformation stance();
};

#endif