#ifndef _QUADRUPPED_ODOMETRY_H_
#define _QUADRUPPED_ODOMETRY_H_

#include <Geometry.h>
#include <quadruped_base.h>

class Velocities
{
    public:
        float linear_velocity_x;
        float linear_velocity_y;
        float angular_velocity_z;
};

class Odometry
{
    QuadrupedBase *base_;
    Transformation prev_foot_position_[4];
    float prev_theta_[4];
    unsigned long int prev_time_;

    public:
        Odometry(QuadrupedBase &quadruped_base);
        void getVelocities(Velocities & vel);
};

#endif