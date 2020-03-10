#ifndef IMU_H
#define IMU_H

#include <imu_plugins.h>

template<typename Plugin>
class IMU
{
    Plugin imu_plugin_;

    public:
        enum Sensor { SimulationIMU, BNO0809DOF};

        IMU()
        {
        }

        void readOrientation(Orientation &rotation)
        {
            imu_plugin_.readOrientation(rotation);
        }

        void readGyroscope(Gyroscope &gyro)
        {
            imu_plugin_.readGyroscope(gyro);
        }

        void readAccelerometer(Accelerometer &accel)
        {
            imu_plugin_.readAccelerometer(accel);
        }

        void readMagnetometer(Magnetometer &mag)
        {
            imu_plugin_.readMagnetometer(mag);
        }
        
        void read(Orientation &rotation, Accelerometer &accel, Gyroscope &gyro, Magnetometer &mag)
        {
            imu_plugin_.read(rotation, accel, gyro, mag);
        }

        void run()
        {
            imu_plugin_.run();
        }
};

#endif

