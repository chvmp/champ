#ifndef RF_INTERFACE_H
#define RF_INTERFACE_H

#include <Arduino.h>
#include <macros/macros.h>

namespace champ
{
    namespace Interfaces
    {
        class RF
        {
            int ele_, rud_, ail_, thr_, aux1_, aux2_;
            int ele_inv_, rud_inv_, ail_inv_, thr_inv_, aux1_inv_, aux2_inv_;
            int inverters_[6];

            champ::Velocities velocities_commands_;
            champ::Pose pose_commands_;

            unsigned long prev_vel_time_;
            unsigned long prev_pose_time_;
            unsigned long prev_poll_time_;

            bool vel_cmd_active_;
            bool pose_cmd_active_;
            public:
                RF(int ele, int rud, int ail, int thr, int aux1, int aux2):
                    ele_(ele),
                    rud_(rud),
                    ail_(ail),
                    thr_(thr),
                    aux1_(aux1),
                    aux2_(aux2),
                    vel_cmd_active_(false),
                    pose_cmd_active_(false)
                {
                }

                void setInverters(bool ele, bool rud, bool ail, bool thr, bool aux1, bool aux2)
                {
                }

                void velocityInput(champ::Velocities &velocities)
                {
                    velocities = velocities_commands_;
                }

                void poseInput(champ::Pose &pose)
                {  
                    pose.roll = pose_commands_.roll;
                    pose.pitch = pose_commands_.pitch;
                    pose.yaw = pose_commands_.yaw;                
                }

                void jointsInput(float joints[12])
                {
     
                }

                bool velInputIsActive()
                {
                    return vel_cmd_active_;
                }

                bool poseInputIsActive()
                {
                    return pose_cmd_active_;            
                }

                bool jointsInputIsActive()
                {
                    return false;           
                }     

                void run()
                {
                    unsigned long now = micros();
                    if((now - prev_poll_time_) > 200000)
                    {
                        prev_poll_time_ = now;

                        vel_cmd_active_ = false;
                        pose_cmd_active_ = false;
                        
                        int rec_aux = pulseIn(aux1_, HIGH, 20000);
                        int rec_ele = pulseIn(ele_, HIGH, 20000);
                        int rec_rud = pulseIn(rud_, HIGH, 20000);
                        int rec_ail = pulseIn(ail_, HIGH, 20000);
                        // float rec_thr = pulseIn(thr_, HIGH, 20000);
                        // float rec_aux2 = pulseIn(aux2_, HIGH, 20000);
                        
                        if(rec_aux < 1500)
                        {
                            velocities_commands_.linear_velocity_x = mapFloat(rec_ele, 1100, 1900, -0.5, 0.5);
                            velocities_commands_.linear_velocity_y = mapFloat(rec_rud, 1100, 1900, -0.5, 0.5);
                            velocities_commands_.angular_velocity_z = mapFloat(rec_ail, 1100, 1900, -1.0, 1.0);
                            vel_cmd_active_ = true;

                            if((rec_ele > 1400 && rec_ele < 1600) &&
                               (rec_rud > 1400 && rec_rud < 1600) && 
                               (rec_ail > 1400 && rec_ail < 1600))
                            {
                                vel_cmd_active_ = false;
                                velocities_commands_.linear_velocity_x = 0.0;
                                velocities_commands_.linear_velocity_y = 0.0;
                                velocities_commands_.angular_velocity_z = 0.0;
                            }

                        }
                        else if(rec_aux > 1500)
                        {
                            pose_commands_.roll = mapFloat(rec_ail, 1100, 1900, -0.523599, 0.523599);
                            pose_commands_.pitch = mapFloat(rec_ele, 1100, 1900, -0.349066, 0.349066);
                            pose_commands_.yaw = mapFloat(rec_rud, 1100, 1900, -0.436332, 0.436332);
                            pose_cmd_active_ = true;

                            if((rec_ele > 1400 && rec_ele < 1600) &&
                               (rec_rud > 1400 && rec_rud < 1600) && 
                               (rec_ail > 1400 && rec_ail < 1600))
                            {
                                pose_cmd_active_ = false;
                                pose_commands_.roll = 0.0;
                                pose_commands_.pitch = 0.0;
                                pose_commands_.yaw = 0.0;
                            }
                        }
                        else if(rec_aux == 0)
                        {
                            vel_cmd_active_ = false;
                            pose_cmd_active_ = false;

                            velocities_commands_.linear_velocity_x = 0.0;
                            velocities_commands_.linear_velocity_y = 0.0;
                            velocities_commands_.angular_velocity_z = 0.0;

                            pose_commands_.roll = 0.0;
                            pose_commands_.pitch = 0.0;
                            pose_commands_.yaw = 0.0;
                        }
                    }
                }
        };
    } 
}

#endif 