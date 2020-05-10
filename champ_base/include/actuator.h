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

namespace champ
    {
        class Actuator
        {
            float thetas_[12];
            float prev_angle_;

            public:
                Actuator()
                {
                }

                void moveJoints(float joint_positions[12])
                {
                    for(unsigned int i = 0; i < 12; i++)
                    {
                        moveJoint(i, joint_positions[i]);
                    }
                }

                void moveJoint(unsigned int leg_id, float joint_position)
                {
                    //until a proper hardware interface is done, hardware integration can be done here
                    
                    //joint_position can be passed to the hardware api to set the joint position of the actuator

                    //this stores the set joint_position by the controller for pseudo feedback.
                    float delta = (joint_position - thetas_[leg_id]);
                    //the random number is just to add noise to the reading
                    thetas_[leg_id] = thetas_[leg_id] + (delta * (((rand() % 80) + 70) / 100.0));  
                }

                void getJointPositions(float joint_position[12])
                {
                    for(unsigned int i = 0; i < 12; i++)
                    {
                        joint_position[i] = getJointPosition(i);
                    }
                }

                float getJointPosition(unsigned int leg_id)
                {    
                    //virtually this returns the stored joint_position set by the controller
                    //until a proper hardware interface is done, this can be used to return
                    //real actuator feedback
                    return thetas_[leg_id];
                }
        };
    }
#endif
