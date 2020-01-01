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
        bool inverted_;
        int inverter_;
        public:
            unsigned int leg_id;
            Plugin(int hardware_pin, float min_angle, float max_angle, bool inverted):
            current_angle_(0),    
            min_angle_(0),
            max_angle_(0),
            inverted_(false),
            inverter_(1),
            leg_id(0)
            {
                delay(100);
                min_angle_ = min_angle;
                max_angle_ = max_angle;
                inverted_ = inverted;
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

                if(angle > 0)
                    actuator_angle = mapFloat(angle, 0, PI, 180, 360);

                else if(angle < 0)
                    actuator_angle = mapFloat(angle, -PI, 0, 0, 180);

                else  
                    actuator_angle = 180;
                
                return round(actuator_angle);
            }

            float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
            {
                return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            }
    };
}

#endif

