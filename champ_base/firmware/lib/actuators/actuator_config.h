#ifndef _ACTUATOR_CONFIG_
#define _ACTUATOR_CONFIG_

class ActuatorConfig
{
    public:
        HardwareSerial serial_interface;
        float angle_offset;
        float min_angle;
        float max_angle;
        bool inverted;
        unsigned int leg_id;

        ActuatorConfig(HardwareSerial &serial_interface, unsigned int actuator_leg_id, unsigned int actuator_driver_id, float min_angle, float max_angle, bool inverted):
        serial_interface(serial_interface),
        angle_offset(0.523599),
        min_angle(0),
        max_angle(0),
        inverted(false),
        leg_id(0)
        {

        }
};

#endif
