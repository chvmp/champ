#ifndef STATUS_H
#define STATUS_H


#include <champ_msgs/Velocities.h>
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

#include <champ_msgs/Pose.h>
#include <champ_msgs/Joints.h>
#include <geometry/geometry.h>

namespace champ
{
    namespace Interfaces
    {
        template<class StatusInterface>

        class Status
        {
                StatusInterface *status_interface_;
                public:
                    
                    Status(StatusInterface &interface):
                        status_interface_(&interface)
                    {
                        
                    }

                    void publishPoints(geometry::Transformation foot_positions[4])
                    {
                        status_interface_->publishPoints(foot_positions);
                    }

                    void publishVelocities(champ::Velocities vel)
                    {
                        status_interface_->publishVelocities(vel);
                    }

                    void publishJointStates(float joint_positions[12])
                    {
                        status_interface_->publishJointStates(joint_positions);
                    }

                    void publishIMU(champ::Quaternion &orientation, champ::Accelerometer &accel, 
                                    champ::Gyroscope &gyro, champ::Magnetometer &mag)
                    {
                        status_interface_->publishIMU(orientation, accel, gyro, mag);
                    }
        };
    }
}
#endif