/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <actuator_plugins.h>

template<typename Plugin>
class Actuator
{
    
    Plugin *actuator_chain_[12];
    bool pangtograph_leg_;
    public:
        enum Driver { Simulation, DynamixelAX12A, Servo, Odrive };

        Actuator(bool pantograph_leg, Plugin &plugin0, Plugin &plugin1, Plugin &plugin2,
                 Plugin &plugin3, Plugin &plugin4, Plugin &plugin5,
                 Plugin &plugin6, Plugin &plugin7, Plugin &plugin8,
                 Plugin &plugin9, Plugin &plugin10, Plugin &plugin11):
        pangtograph_leg_(pantograph_leg)
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
                if(i == 2 || i == 5 || i == 8 || i == 11)
                {
                    if(pangtograph_leg_)
                        actuator_chain_[i]->positionControl(joint_positions[i] + joint_positions[i-1]);
                    else
                        actuator_chain_[i]->positionControl(joint_positions[i]);
                }
                else
                {
                    actuator_chain_[i]->positionControl(joint_positions[i]);
                }
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
                if(i == 2 || i == 5 || i == 8 || i == 11)
                {
                    if(pangtograph_leg_)
                        joint_position[i] = actuator_chain_[i]->getJointPosition() - actuator_chain_[i-1]->getJointPosition();
                    else
                        joint_position[i] = actuator_chain_[i]->getJointPosition();
                }
                else
                {
                    joint_position[i] = actuator_chain_[i]->getJointPosition();
                }

            }
        }

        float getJointPosition(unsigned int leg_id)
        {    
            return actuator_chain_[leg_id]->getJointPosition();
        }
};

#endif

