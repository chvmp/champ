#ifndef _QUADRUPED_GAIT_H_
#define _QUADRUPED_GAIT_H_

#include <Geometry.h>
#include <quadruped_base.h>
#include <trajectory_planner.h>
#include <phase_generator.h>

class QuadrupedGait
{
        QuadrupedBase *base_;

        TrajectoryPlanner *trajectory_planners_[4];

        PhaseGenerator phase_gen_;
        float max_linear_velocity_x_;
        float max_linear_velocity_y_;
        float max_angular_velocity_;
        float stance_duration_;
        float max_theta_;

        void transformTrajectory(QuadrupedLeg *leg, float linear_velocity_x, float linear_velocity_y, float angular_velocity_z, float &step_length, float &rotation);
        float getStepLength(float linear_velocity_x, float linear_velocity_y, float angular_velocity_z);
        float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
        float raibertsHeuristic(float stance_duration, float target_velocity);
    public:
        QuadrupedGait(QuadrupedBase &quadruped_base, float max_linear_velocity_x,float max_linear_velocity_y, float max_angular_velocity, float stance_duration, float swing_height, float stance_depth);
        TrajectoryPlanner lf;
        TrajectoryPlanner rf;
        TrajectoryPlanner lh;
        TrajectoryPlanner rh;

        void generate(Transformation (&foot_positions)[4], float linear_velocity_x, float linear_velocity_y, float angular_velocity_z);
};

#endif