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

#ifndef SINGLE_INPUT_INTERFACE_H
#define SINGLE_INPUT_INTERFACE_H

#include <comms/input_interfaces/rosserial_interface.h>
#include <comms/input_interfaces/rf_interface.h>

namespace champ
{
    namespace Interfaces
    {
        template<class InputInterface>
        class SingleInput
        {
            InputInterface *input_interface_;
            public:
                SingleInput(InputInterface &input_interface):
                    input_interface_(&input_interface)
                {
                }

                void velocityInput(champ::Velocities &velocities)
                {
                    if(input_interface_->velInputIsActive())
                    {
                        input_interface_->velocityInput(velocities);
                    }
                }

                void poseInput(champ::Pose &pose)
                {

                    if(input_interface_->poseInputIsActive())
                    {
                        input_interface_->poseInput(pose);
                    }                    }

                void jointsInput(float joints[12])
                {
                    //make sure no other control input is active before joints control 
                    //is activated
                    if(input_interface_->velInputIsActive() ||
                        input_interface_->poseInputIsActive())
                    {
                        return;
                    }

                    if(input_interface_->jointsInputIsActive())
                    {
                        input_interface_->jointsInput(joints);
                    }                      
                }

                void run()
                {
                    input_interface_->run();              
                }
        };
    }
}
#endif