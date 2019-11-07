#ifndef _DUMMY_ACTUATOR_PLUGIN_H_
#define _DUMMY_ACTUATOR_PLUGIN_H_

namespace DummyActuator
{
    class Plugin
    {
        public:
            Plugin(HardwareSerial &serial_interface, unsigned int actuator_leg_id, unsigned int actuator_driver_id, float min_angle, float max_angle, bool inverted)
            {
                initialize();
            }

            void initialize()
            {
            }

            void positionControl(float angle)
            {
            }

            float getJointPosition()
            {
            }

            int toActuatorAngle(float angle)
            {
            }

            float toEulerAngle(float angle)
            {
            }

    };
}

#endif

