#ifndef DUAL_INPUT_INTERFACE_H
#define DUAL_INPUT_INTERFACE_H

#include <comms/input_interfaces/rosserial_interface.h>
#include <comms/input_interfaces/rf_interface.h>

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