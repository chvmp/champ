#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <quadruped_base/quadruped_base.h>
#include <geometry/geometry.h>

namespace champ
{
    class Odometry
    {
        QuadrupedBase *base_;
        Transformation prev_foot_position_[4];
        bool prev_gait_phase_[4];
        float prev_theta_[4];
        unsigned long int prev_time_;
        int theta_direction_[4];

        public:
            Odometry(QuadrupedBase &quadruped_base):
                base_(&quadruped_base),
                prev_gait_phase_{1,1,1,1},
                prev_theta_{0,0,0,0},
                prev_time_(0),
                theta_direction_{1,1,-1,-1}
            {
                for(unsigned int i = 0; i < 4; i++)
                {
                    prev_foot_position_[i] = base_->legs[i]->foot_from_hip();
                }
            }        
            
            void getVelocities(champ::Velocities &vel)
            {
                unsigned int total_contact = 0;
                float x_sum = 0;
                float y_sum = 0;
                float theta_sum = 0;

                for(unsigned int i = 0; i < 4; i++)
                {
                    Transformation current_foot_position = base_->legs[i]->foot_from_base();
                
                    bool current_gait_phase = base_->legs[i]->gait_phase();
                    
                    float delta_x = (prev_foot_position_[i].X() - current_foot_position.X());
                    float delta_y = (prev_foot_position_[i].Y() - current_foot_position.Y());
                    
                    float current_theta = atan2f(current_foot_position.Y(), current_foot_position.X());
                    float delta_theta = (prev_theta_[i] - current_theta);

                    if(current_gait_phase && prev_gait_phase_[i])
                    {
                        total_contact += 1;
                
                        x_sum += delta_x;
                        y_sum += delta_y;
                        theta_sum += delta_theta;
                    }

                    prev_foot_position_[i] = current_foot_position;
                    prev_gait_phase_[i] = current_gait_phase;
                    prev_theta_[i] = current_theta;
                }

                if(total_contact > 1)
                {
                    x_sum /= total_contact;
                    y_sum /= total_contact;
                    theta_sum /= total_contact;
                }

                unsigned long int now = time_us();
                double dt = (now - prev_time_) / 1000000.0;
                
                vel.linear_velocity_x = x_sum / dt;
                vel.linear_velocity_y = y_sum / dt;
                vel.angular_velocity_z = theta_sum / dt;
                
                prev_time_ = now;
            }
    };
}

#endif