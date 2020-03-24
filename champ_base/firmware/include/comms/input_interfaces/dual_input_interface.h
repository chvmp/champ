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

#ifndef DUAL_INPUT_INTERFACE_H
#define DUAL_INPUT_INTERFACE_H

namespace champ
{
    namespace Interfaces
    {
        template<class SoftwareInterface, class HardwareInterface>
        class DualInput
        {
            SoftwareInterface *software_interface_;
            HardwareInterface *hardware_interface_;
            unsigned int total_interface_;
            public:
                DualInput(SoftwareInterface &software_interface, 
                                HardwareInterface &hardware_interface):
                    software_interface_(&software_interface),
                    hardware_interface_(&hardware_interface)
                {
                }

                void velocityInput(champ::Velocities &velocities)
                {
                    if(hardware_interface_->velInputIsActive())
                    {
                        hardware_interface_->velocityInput(velocities);
                    }
                    else if(software_interface_->velInputIsActive())
                    {
                        software_interface_->velocityInput(velocities);
                    }
                }

                void poseInput(champ::Pose &pose)
                {
                    if(hardware_interface_->poseInputIsActive())
                    {
                        hardware_interface_->poseInput(pose);
                    }
                    else if(software_interface_->poseInputIsActive())
                    {
                        software_interface_->poseInput(pose);
                    }              
                }

                void jointsInput(float joints[12])
                {
                    //make sure no other control input is active before joints control 
                    //is activated
                    if(hardware_interface_->velInputIsActive()   ||
                        software_interface_->velInputIsActive()   || 
                        hardware_interface_->poseInputIsActive()  ||
                        software_interface_->poseInputIsActive())
                    {
                        return;
                    }

                    if(software_interface_->jointsInputIsActive())
                    {
                        software_interface_->jointsInput(joints);
                    }            
                }

                void run()
                {
                    software_interface_->run();
                    hardware_interface_->run();              
                }
        };
    }
}
#endif