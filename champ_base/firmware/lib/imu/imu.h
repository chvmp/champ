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

        void readOrientation(champ::Quaternion &orientation)
        {
            imu_plugin_.readOrientation(orientation);
        }

        void readGyroscope(champ::Gyroscope &gyro)
        {
            imu_plugin_.readGyroscope(gyro);
        }

        void readAccelerometer(champ::Accelerometer &accel)
        {
            imu_plugin_.readAccelerometer(accel);
        }

        void readMagnetometer(champ::Magnetometer &mag)
        {
            imu_plugin_.readMagnetometer(mag);
        }
        
        void read(champ::Quaternion &orientation, champ::Accelerometer &accel, champ::Gyroscope &gyro, champ::Magnetometer &mag)
        {
            imu_plugin_.read(orientation, accel, gyro, mag);
        }

        void run()
        {
            imu_plugin_.run();
        }
};

#endif

