#ifndef RF_INTERFACE_H
#define RF_INTERFACE_H

#include <Arduino.h>

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
                    // pinMode(ele_, INPUT);
                    // pinMode(rud_, INPUT);
                    // pinMode(ail_, INPUT);
                    // pinMode(thr_, INPUT);
                    // pinMode(aux1_, INPUT);
                    // pinMode(aux2_, INPUT);
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
                    pose = pose_commands_;
                }

                void jointsInput(float joints[12])
                {
     
                }

                bool velInputIsActive()
                {
                    return true;
                }

                bool poseInputIsActive()
                {
                    return pose_cmd_active_;            
                }

                bool jointsInputIsActive()
                {
                    return false;           
                }

                float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
                {
                    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
                }

                void run()
                {
                    unsigned long now = micros();
                    if((now - prev_poll_time_) > 333333)
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
                            velocities_commands_.linear_velocity_x = mapfloat(rec_ele, 1100, 1900, -0.2, 0.2);
                            velocities_commands_.linear_velocity_y = mapfloat(rec_rud, 1100, 1900, -0.2, 0.2);
                            velocities_commands_.angular_velocity_z = mapfloat(rec_ail, 1100, 1900, -1.0, 1.0);
                            vel_cmd_active_ = true;

                            // if(rec_ele > 1400 && rec_ele < 1600)
                            // {
                            //     vel_cmd_active_ = false;
                            // }

                            // else if(rec_rud > 1400 && rec_rud < 1600)
                            // {
                            //     vel_cmd_active_ = false;
                            // }

                            // else if(rec_ail > 1400 && rec_ail < 1600)
                            // {
                            //     vel_cmd_active_ = false;
                            // }
                        }
                        else if(rec_aux > 1500)
                        {
                            // pose_commands_.roll = mapfloat(rec_ele, 1100, 1900, -0.5, 0.5);
                            // pose_commands_.pitch = mapfloat(rec_ail, 1100, 1900, -0.5, 0.5);
                            // pose_commands_.yaw = mapfloat(rec_rud, 1100, 1900, -1.0, 1.0);
                            // pose_cmd_active_ = true;

                            // if(rec_ele > 1400 && rec_ele < 1600)
                            // {
                            //     pose_cmd_active_ = false;
                            // }

                            // else if(rec_ail > 1400 && rec_ail < 1600)
                            // {
                            //     pose_cmd_active_ = false;
                            // }

                            // else if(rec_rud > 1400 && rec_rud < 1600)
                            // {
                            //     pose_cmd_active_ = false;
                            // }
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