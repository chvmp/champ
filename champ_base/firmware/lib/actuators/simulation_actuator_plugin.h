#ifndef _SIMULATION_ACTUATOR_PLUGIN_H_
#define _SIMULATION_ACTUATOR_PLUGIN_H_

namespace SimulationActuator
{
    class Plugin
    {
        float angle_;
        public:
            Plugin():
            angle_(0)
            {
                initialize();
            }

            void initialize()
            {
            }

            void positionControl(float angle)
            {
                angle_ = angle;
            }

            float getJointPosition()
            {
                return angle_;
            }
    };
}

#endif

