#ifndef SIMULATION_ACTUATOR_PLUGIN_H
#define SIMULATION_ACTUATOR_PLUGIN_H

namespace SimulationActuator
{
    class Plugin
    {
        float prev_angle_;
        float angle_;
        float noise_;
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
                float delta = (angle - angle_);
                angle_ = angle_ + (delta * (random(70, 80) / 100.0));
            }

            float getJointPosition()
            {
                return angle_;
            }
    };
}

#endif

