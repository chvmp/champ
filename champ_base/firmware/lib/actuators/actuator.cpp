#include <actuator.h>

Actuator::Actuator()
{

}

void Actuator::add(unsigned int leg_id, DynamixelAX12A::ActuatorPlugin &actuator_plugin)
{
    actuator_chain_[leg_id] = &actuator_plugin;
}

void Actuator::moveJoints(float joint_positions[12])
{
    for(unsigned int i = 0; i < 12; i++)
    {
        actuator_chain_[i]->positionControl(joint_positions[i]);
    }
}

void Actuator::moveJoint(unsigned int leg_id, float joint_position)
{
    actuator_chain_[leg_id]->positionControl(joint_position);
}

void Actuator::getJointPositions(float joint_position[12])
{
    for(unsigned int i = 0; i < 12; i++)
    {
        joint_position[i] = actuator_chain_[i]->getJointPosition();
    }
}

float Actuator::getJointPosition(unsigned int leg_id)
{    
    return actuator_chain_[leg_id]->getJointPosition();;
}