#ifndef _IMU_H_
#define _IMU_H_

#include <imu_plugins.h>
#include <imu_components.h>

template<typename Plugin>
class IMU
{
    Plugin imu_plugin_;

    public:
        enum Sensor {BNO0809DOF};

        IMU()
        {
        }

        void readGyroscope(Gyroscope &gyro)
        {
            gyro.x = imu_plugin_.readGyroscopeX();
            gyro.y = imu_plugin_.readGyroscopeY();
            gyro.z = imu_plugin_.readGyroscopeZ();
        }

        void readAccelerometer(Accelerometer &accel)
        {
            accel.x = imu_plugin_.readAccelerometerX();
            accel.y = imu_plugin_.readAccelerometerY();
            accel.z = imu_plugin_.readAccelerometerZ();
        }

        void readMagnetometer(Magnetometer &mag)
        {
            mag.x = imu_plugin_.readMagnetometerX();
            mag.y = imu_plugin_.readMagnetometerY();
            mag.z = imu_plugin_.readMagnetometerZ();
        }
};

#endif

