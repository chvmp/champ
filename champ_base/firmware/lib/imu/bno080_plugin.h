#ifndef BNO080_PLUGIN_H
#define BNO080_PLUGIN_H

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <quadruped_base/quadruped_components.h>

namespace BNO0809DOF
{   
    class Plugin
    {
        BNO080 imu_;
        champ::Orientation rotation_;
        champ::Gyroscope gyro_;
        champ::Accelerometer accel_;
        champ::Magnetometer mag_;

        public:
            Plugin()
            {
                initialize();
            }

            void initialize()
            {
                delay(100);

                Wire.begin();
                imu_.begin(0x4B, Wire, 15);

                Wire.setClock(400000);
                imu_.enableRotationVector(50);
                imu_.enableAccelerometer(50);
                imu_.enableGyro(50); 
                imu_.enableMagnetometer(50);
            }

            void readchamp(champ::Orientation &rotation)
            {
                if(imu_.dataAvailable())
                {
                    rotation.w = imu_.getQuatReal();
                    rotation.x = imu_.getQuatI();
                    rotation.y = imu_.getQuatJ();
                    rotation.z = imu_.getQuatK();

                    // to euler
                    //https://stackoverflow.com/questions/30279065/how-to-get-the-euler-angles-from-the-rotation-vector-sensor-type-rotation-vecto
                    // rotation.x  = atan2f(-2.* (q[2] * q[3] - q[0] * q[1]) , q[0] * q[0] - q[1] * q[1]- q[2] * q[2] + q[3] * q[3]); 
                    // rotation.y  = asinf(2. * (q[1] * q[3] + q[0] * q[2]));
                    // rotation.z  = atan2f( 2. * (-q[1] * q[2] + q[0] * q[3]) , q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); 
                }   
            }

            void readGyroscope(champ::Gyroscope &gyro)
            {
                if(imu_.dataAvailable())
                {
                    gyro.x = imu_.getGyroX();
                    gyro.y = imu_.getGyroY();
                    gyro.z = imu_.getGyroZ();
                } 
            }

            void readAccelerometer(champ::Accelerometer &accel)
            {
                if(imu_.dataAvailable())
                {
                    accel.x = imu_.getAccelX();
                    accel.y = imu_.getAccelY();
                    accel.z = imu_.getAccelZ();
                }
            }

            void readMagnetometer(champ::Magnetometer &mag)
            {
                if(imu_.dataAvailable())
                {
                    mag.x = imu_.getMagX();
                    mag.y = imu_.getMagY();
                    mag.z = imu_.getMagZ();
                }
            }

            void read(champ::Orientation &rotation, champ::Accelerometer &accel, champ::Gyroscope &gyro, champ::Magnetometer &mag)
            {
                rotation.w = rotation_.w;
                rotation.x = rotation_.x;
                rotation.y = rotation_.y;
                rotation.z = rotation_.z;

                gyro.x = gyro_.x;
                gyro.y = gyro_.y;
                gyro.z = gyro_.z;

                accel.x = accel_.x;
                accel.y = accel_.y;
                accel.z = accel_.z;

                mag.x = mag_.x;
                mag.y = mag_.y;
                mag.z = mag_.z;
            }

            void run()
            {
                if(imu_.dataAvailable())
                {
                    rotation_.w = imu_.getQuatReal();
                    rotation_.x = imu_.getQuatI();
                    rotation_.y = imu_.getQuatJ();
                    rotation_.z = imu_.getQuatK();

                    gyro_.x = imu_.getGyroX();
                    gyro_.y = imu_.getGyroY();
                    gyro_.z = imu_.getGyroZ();

                    accel_.x = imu_.getAccelX();
                    accel_.y = imu_.getAccelY();
                    accel_.z = imu_.getAccelZ();

                    mag_.x = imu_.getMagX();
                    mag_.y = imu_.getMagY();
                    mag_.z = imu_.getMagZ();
                }
            }
    };
}

#endif

