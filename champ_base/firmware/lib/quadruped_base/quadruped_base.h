#ifndef _QUADRUPPED_BASE_H_
#define _QUADRUPPED_BASE_H_

#include<quadruped_leg.h>

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

class QuadrupedBase
{   
    Velocities speed_;
    Attitude attitude_;

    const char * knee_orientation_;
    
    int getKneeDirection(char direction);
    
    public:
        QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg, const char *knee_orientation);
        void getJointPositions(float *joint_positions);
        void getFootPositions(Transformation *foot_positions);
        void updateJointPositions(float joints_states[12]);
        
        float roll();
        void roll(float roll);

        float pitch();
        void pitch(float pitch);

        float yaw();
        void yaw(float yaw);

        void updateAttitude(Attitude attitude);
        void updateAttitude(float roll,  float pitch, float yaw);
        Attitude attitude();
        
        float linear_velocity_x();
        void linear_velocity_x(float linear_velocity_x);

        float linear_velocity_y();
        void linear_velocity_y(float linear_velocity_y);

        float angular_velocity_z();
        void angular_velocity_z(float angular_velocity_z);

        void updateSpeed(Velocities speed);
        void updateSpeed(float linear_velocity_x,  float linear_velocity_y, float angular_velocity_z);
        Velocities speed();


        QuadrupedLeg *legs[4];

        QuadrupedLeg *lf;
        QuadrupedLeg *rf;
        QuadrupedLeg *lh;
        QuadrupedLeg *rh;
};

#endif


