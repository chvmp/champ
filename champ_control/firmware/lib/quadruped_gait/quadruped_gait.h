#ifndef _QUADRUPED_GAIT_H_
#define _QUADRUPED_GAIT_H_

#include<Arduino.h>
#include<Geometry.h>
#include<quadruped_base.h>
#include<trajectory_planner.h>
#include<phase_generator.h>

class QuadrupedGait
{
        QuadrupedBase *base_;

        TrajectoryPlanner *trajectory_planners_[4];

        PhaseGenerator phase_gen_;

    public:
        QuadrupedGait(QuadrupedBase &quadruped_base, float max_velocity, float swing_height, float step_length, float stance_depth);

        TrajectoryPlanner lf;
        TrajectoryPlanner rf;
        TrajectoryPlanner lh;
        TrajectoryPlanner rh;

        void generate(Transformation (&foot_positions)[4], float linear_velocity_x, float linear_velocity_y, float angular_velocity_z);
};

#endif