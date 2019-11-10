#include <odometry.h>

Odometry::Odometry(QuadrupedBase &quadruped_base):
    base_(&quadruped_base),
    prev_theta_{0,0,0,0},
    prev_time_(0)
{
    for(unsigned int i = 0; i < 4; i++)
    {
        prev_foot_position_[i] = base_->legs[i]->foot_from_hip();
    }
}

void Odometry::getVelocities(Velocities &vel)
{
    float x_sum = 0;
    float y_sum = 0;
    float theta_sum = 0;

    for(unsigned int i = 0; i < 4; i++)
    {
        Transformation current_foot_position = base_->legs[i]->foot_from_base();
        
        float current_theta = atan2(current_foot_position.Y(), current_foot_position.X());
        
        if(base_->legs[i]->gait_phase())
        {
            float delta_x = (prev_foot_position_[i].X() - current_foot_position.X()) / 2;
            float delta_y = (prev_foot_position_[i].Y() - current_foot_position.Y()) / 2;
            float delta_theta = prev_theta_[i] - current_theta;

            x_sum += delta_x;
            y_sum += delta_y;
            theta_sum += delta_theta;

            prev_foot_position_[i] = current_foot_position;
            prev_theta_[i] = current_theta;
        }
    }
    
    unsigned long int now = micros();
    double dt = (now - prev_time_) / 1000000;
    
    vel.linear_velocity_x = x_sum / dt;
    vel.linear_velocity_y = y_sum / dt;
    vel.angular_velocity_z = theta_sum / dt;
    
    prev_time_ = now;
}
