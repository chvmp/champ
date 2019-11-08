#ifndef _BNO080_PLUGIN_H_
#define _BNO080_PLUGIN_H_

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <imu_components.h>

namespace BNO0809DOF
{   
    class Plugin
    {
        BNO080 imu_;
        public:
            Plugin()
            {
                initialize();
            }

            void initialize()
            {
                Wire.begin();
                imu_.begin();
                Wire.setClock(400000);
                imu_.enableRotationVector(8);
                imu_.enableAccelerometer(8);
                imu_.enableGyro(8); 
                imu_.enableMagnetometer(8);
            }

            void readOrientation(Orientation &rotation)
            {
                if(imu_.dataAvailable())
                {
                    rotation.w = imu_.getQuatReal();
                    rotation.x = imu_.getQuatI();
                    rotation.y = imu_.getQuatJ();
                    rotation.z = imu_.getQuatK();

                    // to euler
                    //https://stackoverflow.com/questions/30279065/how-to-get-the-euler-angles-from-the-rotation-vector-sensor-type-rotation-vecto
                    // rotation.x  = atan2(-2.* (q[2] * q[3] - q[0] * q[1]) , q[0] * q[0] - q[1] * q[1]- q[2] * q[2] + q[3] * q[3]); 
                    // rotation.y  = asin(2. * (q[1] * q[3] + q[0] * q[2]));
                    // rotation.z  = atan2( 2. * (-q[1] * q[2] + q[0] * q[3]) , q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); 
                }
                else
                {
                    rotation.w = 0.0;
                    rotation.x = 0.0;
                    rotation.y = 0.0;
                    rotation.z = 0.0;
                }    
            }

            void readGyroscope(Gyroscope &gyro)
            {
                if(imu_.dataAvailable())
                {
                    gyro.x = imu_.getGyroX();
                    gyro.y = imu_.getGyroY();
                    gyro.z = imu_.getGyroZ();
                }
                else
                {
                    gyro.x = 0.0;
                    gyro.y = 0.0;
                    gyro.z = 0.0;
                }    
            }

            void readAccelerometer(Accelerometer &accel)
            {
                if(imu_.dataAvailable())
                {
                    accel.x = imu_.getAccelX();
                    accel.y = imu_.getAccelY();
                    accel.z = imu_.getAccelZ();
                }
                else
                {
                    accel.x = 0.0;
                    accel.y = 0.0;
                    accel.z = 0.0; 
                }
            }

            void readMagnetometer(Magnetometer &mag)
            {
                if(imu_.dataAvailable())
                {
                    mag.x = imu_.getMagX();
                    mag.y = imu_.getMagY();
                    mag.z = imu_.getMagZ();
                }
                else
                {
                    mag.x = 0.0;
                    mag.y = 0.0;
                    mag.z = 0.0; 
                } 
            }
    };
}

#endif

