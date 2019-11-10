#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include <actuator_plugins.h>

template<typename Plugin>
class Actuator
{
    
    Plugin *actuator_chain_[12];

    public:
        enum Driver { Simulation, DynamixelAX12A, Servo, Odrive };

        Actuator(Plugin &plugin0, Plugin &plugin1, Plugin &plugin2,
                 Plugin &plugin3, Plugin &plugin4, Plugin &plugin5,
                 Plugin &plugin6, Plugin &plugin7, Plugin &plugin8,
                 Plugin &plugin9, Plugin &plugin10, Plugin &plugin11)
        {
            add(0, plugin0);
            add(1, plugin1);
            add(2, plugin2);
            add(3, plugin3);
            add(4, plugin4);
            add(5, plugin5);
            add(6, plugin6);
            add(7, plugin7);
            add(8, plugin8);
            add(9, plugin9);
            add(10, plugin10);
            add(11, plugin11);
        }

        void add(unsigned int leg_id, Plugin &actuator_plugin)
        {
            actuator_chain_[leg_id] = &actuator_plugin;
        }

        void moveJoints(float joint_positions[12])
        {
            for(unsigned int i = 0; i < 12; i++)
            {
                actuator_chain_[i]->positionControl(joint_positions[i]);
            }
        }

        void moveJoint(unsigned int leg_id, float joint_position)
        {
            actuator_chain_[leg_id]->positionControl(joint_position);
        }

        void getJointPositions(float joint_position[12])
        {
            for(unsigned int i = 0; i < 12; i++)
            {
                joint_position[i] = actuator_chain_[i]->getJointPosition();
            }
        }

        float getJointPosition(unsigned int leg_id)
        {    
            return actuator_chain_[leg_id]->getJointPosition();
        }
};

#endif

