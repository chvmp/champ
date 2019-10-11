#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include<plugins.h>

class Actuator
{
    ActuatorPlugin *actuator_chain_[12];

    public:
        Actuator();
        void add(unsigned int leg_id, ActuatorPlugin &actuator_plugin);
        void moveJoints(float joint_positions[12]);
        void moveJoint(unsigned int leg_id, float joint_position);

        void getJointPositions(float joint[12]);
        float getJointPosition(unsigned int leg_id);
};

#endif

