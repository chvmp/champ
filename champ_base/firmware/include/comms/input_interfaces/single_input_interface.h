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