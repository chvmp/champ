/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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

    class Quaternion
    {
        public:
            Quaternion():
                x(0.0f), 
                y(0.0f), 
                z(0.0f),
                w(0.0f)
            {}
            float x;
            float y;
            float z;
            float w;
    };

    class Point
    {
        public:
            float x;
            float y;
            float z;
            Point():
                x(0.0f),
                y(0.0f),
                z(0.0f)
            {}
    };

    class Euler
    {
        public:
            Euler():
                roll(0.0f), 
                pitch(0.0f), 
                yaw(0.0f)
            {}
            float roll;
            float pitch;
            float yaw;
    };

    class Pose
    {
        public:
            Point translation;
            Euler orientation;
    };

    class Accelerometer
    {
        public:
            Accelerometer():
                x(0.0f), 
                y(0.0f), 
                z(0.0f)
            {}
            float x;
            float y;
            float z;
    };

    class Gyroscope
    {
        public:
            Gyroscope():
                x(0.0f), 
                y(0.0f), 
                z(0.0f)
            {}
            float x;
            float y;
            float z;
    };

    class Magnetometer
    {
        public:
            Magnetometer():
                x(0.0f), 
                y(0.0f), 
                z(0.0f)
            {}
            float x;
            float y;
            float z;
    };

    class GaitConfig
    {
        public:
            GaitConfig():
                knee_orientation(">>"),
                pantograph_leg(false),
                max_linear_velocity_x(0.0f),
                max_linear_velocity_y(0.0f),
                max_angular_velocity_z(0.0f),
                swing_height(0.0f),
                stance_depth(0.0f),
                stance_duration(0.0f),
                nominal_height(0.0f)
            {}
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
            {}
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