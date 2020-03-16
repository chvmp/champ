#ifndef QUADRUPED_COMPONENTS_H
#define QUADRUPED_COMPONENTS_H

namespace champ
{
    class Attitude
    {
        public:
            Attitude():
                roll(0.0),
                pitch(0.0),
                yaw(0.0)
            {}
            float roll;
            float pitch;
            float yaw;
    };

    class Velocities
    {
        public:
            Velocities():
                linear_velocity_x(0.0),
                linear_velocity_y(0.0),
                angular_velocity_z(0.0)
            {}
            float linear_velocity_x;
            float linear_velocity_y;
            float angular_velocity_z;
    };

    class Orientation
    {
        public:
            Orientation():
                x(0.0), 
                y(0.0), 
                z(0.0)
            {}
            float x;
            float y;
            float z;
            float w;
    };

    class Pose
    {
        public:
            Pose():
                x(0.0), 
                y(0.0), 
                z(0.0), 
                roll(0.0), 
                pitch(0.0), 
                yaw(0.0)
            {}
            float x;
            float y;
            float z;
            float roll;
            float pitch;
            float yaw;
    };

    class Accelerometer
    {
        public:
            Accelerometer():
                x(0.0), 
                y(0.0), 
                z(0.0)
            {}
            float x;
            float y;
            float z;
    };

    class Gyroscope
    {
        public:
            Gyroscope():
                x(0.0), 
                y(0.0), 
                z(0.0)
            {}
            float x;
            float y;
            float z;
    };

    class Magnetometer
    {
        public:
            Magnetometer():
                x(0.0), 
                y(0.0), 
                z(0.0)
            {}
            float x;
            float y;
            float z;
    };
}

#endif