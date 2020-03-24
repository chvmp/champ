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

#ifndef SIMULATION_IMU_PLUGIN_H
#define SIMULATION_IMU_PLUGIN_H

#include <quadruped_base/quadruped_components.h>

namespace SimulationIMU
{   
    class Plugin
    {
        public:
            Plugin()
            {
                initialize();
            }

            void initialize()
            {
            }

            void readOrientation(champ::Quaternion &orientation)
            {
                orientation.w = 1.0;
                orientation.x = 0.0;
                orientation.y = 0.0;
                orientation.z = 0.0;
            }

            void readGyroscope(champ::Gyroscope &gyro)
            {
                gyro.x = 0.0;
                gyro.y = 0.0;
                gyro.z = 0.0;
            }

            void readAccelerometer(champ::Accelerometer &accel)
            {
                accel.x = 0.0;
                accel.y = 0.0;
                accel.z = 0.0; 
            }

            void readMagnetometer(champ::Magnetometer &mag)
            {
                mag.x = 0.0;
                mag.y = 0.0;
                mag.z = 0.0; 
            }

            void read(champ::Quaternion &orientation, champ::Accelerometer &accel, champ::Gyroscope &gyro, champ::Magnetometer &mag)
            {
                orientation.w = 0.0;
                orientation.x = 0.0;
                orientation.y = 0.0;
                orientation.z = 0.0;

                gyro.x = 0.0;
                gyro.y = 0.0;
                gyro.z = 0.0;

                accel.x = 0.0;
                accel.y = 0.0;
                accel.z = 0.0;

                mag.x = 0.0;
                mag.y = 0.0;
                mag.z = 0.0;
            }

            void run()
            {
                
            }
    };
}

#endif

