#ifndef _PWM_SERVO_PLUGIN_H_
#define _PWM_SERVO_PLUGIN_H_

#include <Servo.h>

namespace DigitalServo
{
    class Plugin
    {
        Servo servo_;
        float current_angle_;
        float min_angle_;
        float max_angle_;
        float min_actuator_angle_;
        float max_actuator_angle_;
        bool inverted_;
        int inverter_;
        int offset_;
        public:
            unsigned int leg_id;
            Plugin(int hardware_pin, int max_actuator_angle, int offset, bool inverted):
            current_angle_(0),    
            // min_angle_(min_angle),
            // max_angle_(max_angle),
            max_actuator_angle_(max_actuator_angle),
            inverted_(inverted),
            inverter_(1),
            offset_(offset),
            leg_id(0)
            {
                delay(100);
            
                if(inverted_)
                    inverter_ *= -1;
                
                servo_.attach(hardware_pin);
            }

            void positionControl(float angle)
            {
                current_angle_ = angle;
                servo_.write(toActuatorAngle(angle * inverter_));
            }

            float getJointPosition()
            {
                return current_angle_;
            }

            int toActuatorAngle(float angle)
            {
                float actuator_angle = 0;

                if(max_actuator_angle_ == 180)
                {
                    actuator_angle = mapFloat(angle, -1.134464, 2.007124, 0.0, 180.0);
                    actuator_angle =  round(actuator_angle) + offset_;

                    // if(angle > 0.0)
                    // {
                    //     actuator_angle = mapFloat(angle, -0.785398, 2.35619, 0.0, 180.0);
                    // }
                    // else if(angle < 0.0)
                    // {
                    //     actuator_angle = mapFloat(angle, -2.35619, 0.785398, 0.0, 180.0);
                    // }
                    // else
                    // {
                        
                    // }
                    
                }
                else if(max_actuator_angle_ == 270)
                {
                    actuator_angle = mapFloat(angle, -(PI / 2.0) , PI, 0.0, 180.0);
                                    int offset = map(actuator_angle, 0, 270, 3, 17.5);
                    actuator_angle =  round(actuator_angle) + offset;
                }
                else if(max_actuator_angle_ == 360)
                {
                    actuator_angle = mapFloat(angle, -PI, PI, 0.0, 180.0);
                }

                return actuator_angle;
            }

            float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
            {
                return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            }
    };
}

#endif

