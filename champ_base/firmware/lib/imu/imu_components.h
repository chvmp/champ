#ifndef IMU_COMPONENTS_H
#define IMU_COMPONENTS_H

class Orientation
{
    public:
        float x;
        float y;
        float z;
        float w;
};

class Accelerometer
{
    public:
        float x;
        float y;
        float z;
};

class Gyroscope
{
    public:
        float x;
        float y;
        float z;
};

class Magnetometer
{
    public:
        float x;
        float y;
        float z;
};

#endif