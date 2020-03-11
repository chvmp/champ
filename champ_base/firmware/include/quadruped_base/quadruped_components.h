#ifndef QUADRUPED_COMPONENTS_H
#define QUADRUPED_COMPONENTS_H

namespace champ
{
    class Attitude
    {
        public:
            float roll;
            float pitch;
            float yaw;
    };

    class Velocities
    {
        public:
            float linear_velocity_x;
            float linear_velocity_y;
            float angular_velocity_z;
    };

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
}

#endif