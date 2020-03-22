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

