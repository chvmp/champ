#ifndef SIMULATION_IMU_PLUGIN_H
#define SIMULATION_IMU_PLUGIN_H

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

            void readOrientation(Orientation &rotation)
            {
                rotation.w = 1.0;
                rotation.x = 0.0;
                rotation.y = 0.0;
                rotation.z = 0.0;
            }

            void readGyroscope(Gyroscope &gyro)
            {
                gyro.x = 0.0;
                gyro.y = 0.0;
                gyro.z = 0.0;
            }

            void readAccelerometer(Accelerometer &accel)
            {
                accel.x = 0.0;
                accel.y = 0.0;
                accel.z = 0.0; 
            }

            void readMagnetometer(Magnetometer &mag)
            {
                mag.x = 0.0;
                mag.y = 0.0;
                mag.z = 0.0; 
            }

            void read(Orientation &rotation, Accelerometer &accel, Gyroscope &gyro, Magnetometer &mag)
            {
                rotation.w = 0.0;
                rotation.x = 0.0;
                rotation.y = 0.0;
                rotation.z = 0.0;

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

