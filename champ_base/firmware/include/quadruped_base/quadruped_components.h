#ifndef QUADRUPED_COMPONENTS_H
#define QUADRUPED_COMPONENTS_H

namespace champ
{
    class Linear
    {
        public:
            float x;
            float y;
            float z;
            Linear():
                x(0.0f),
                y(0.0f),
                z(0.0f)
            {}
    };

    class Angular
    {
        public:
            float x;
            float y;
            float z;
            Angular():
                x(0.0f),
                y(0.0f),
                z(0.0f)
            {}
    };

    class Velocities: public Linear, public Angular
    {
        public:
            Linear linear;
            Angular angular;
    };

    // class Velocities
    // {
    //     public:
    //         Velocities():
    //             linear_velocity_x(0.0),
    //             linear_velocity_y(0.0),
    //             angular_velocity_z(0.0)
    //         {}
    //         float linear_velocity_x;
    //         float linear_velocity_y;
    //         float angular_velocity_z;
    // };

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

    class GaitConfig
    {
        public:
            GaitConfig();
            GaitConfig(const char * knee_or,
                bool panto_leg,
                float max_l_x,
                float max_l_y,
                float max_a_z,
                float swing_h,
                float stan_dep,
                float stan_dur,
                float nom_height):
                    knee_orientation(knee_or),
                    pantograph_leg(panto_leg),
                    max_linear_velocity_x(max_l_x),
                    max_linear_velocity_y(max_l_y),
                    max_angular_velocity_z(max_a_z),
                    swing_height(swing_h),
                    stance_depth(stan_dep),
                    stance_duration(stan_dur),
                    nominal_height(nom_height)
            {
            };
            const char * knee_orientation;
            bool pantograph_leg;
            float max_linear_velocity_x;
            float max_linear_velocity_y;
            float max_angular_velocity_z;
            float swing_height;
            float stance_depth;
            float stance_duration;
            float nominal_height;
    };
}

#endif