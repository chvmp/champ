#ifndef _QUADRUPPED_ODOMETRY_H_
#define _QUADRUPPED_ODOMETRY_H_

#include <Geometry.h>
#include <quadruped_base.h>

class Odometry
{
    QuadrupedBase *base_;
    Transformation prev_foot_position_[4];
    bool prev_gait_phase_[4];
    float prev_theta_[4];
    unsigned long int prev_time_;
    int theta_direction_[4];

    public:
        Odometry(QuadrupedBase &quadruped_base);
        void getVelocities(Velocities & vel);
};

#endif