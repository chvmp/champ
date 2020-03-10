#ifndef QUADRUPPED_BASE_H
#define QUADRUPPED_BASE_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>
#include <quadruped_base/quadruped_leg.h>

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
    
    int getKneeDirection(char direction)
    {
        switch (direction) 
        {
            case '>':
                return -1;
            case '<':
                return 1;
            default:
                return -1;
        }
    }    

    public:
        QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg, const char *knee_orientation, bool pantograph_leg):        
            lf(&lf_leg),
            rf(&rf_leg),
            lh(&lh_leg),
            rh(&rh_leg)
        {
            unsigned int total_legs = 0;

            legs[total_legs++] = lf;
            legs[total_legs++] = rf;
            legs[total_legs++] = lh;
            legs[total_legs++] = rh;

            for(unsigned int i=0; i < 4; i++)
            {
                int dir;
                legs[i]->leg_id(i);
                if(i < 2)
                {
                    dir = getKneeDirection(knee_orientation[0]);
                }
                else
                {
                    dir = getKneeDirection(knee_orientation[1]);
                }
                legs[i]->is_pantograph(pantograph_leg);
                legs[i]->knee_direction(dir);
            }
        }        
        
        void getJointPositions(float *joint_positions)
        {
            unsigned int total_joints = 0;

            for(unsigned int i = 0; i < 4; i++)
            {
                joint_positions[total_joints++] = legs[i]->hip->theta();
                joint_positions[total_joints++] = legs[i]->upper_leg->theta();
                joint_positions[total_joints++] = legs[i]->lower_leg->theta();
            }
        }

        void getFootPositions(Transformation *foot_positions)
        {
            for(unsigned int i = 0; i < 4; i++)
            {
                foot_positions[i] = legs[i]->foot_from_base();
            }
        }

        void updateJointPositions(float joints[12])
        {
            for(unsigned int i = 0; i < 4; i++)
            {
                int index = i * 3;
                legs[i]->hip->theta(joints[index]);
                legs[i]->upper_leg->theta(joints[index + 1]);
                legs[i]->lower_leg->theta(joints[index + 2]);
            }
        }
        
        float roll()
        {
            return attitude_.roll;
        }

        void roll(float roll)
        {
            attitude_.roll = roll;
        }

        float pitch()
        {
            return attitude_.pitch;
        }

        void pitch(float pitch)
        {
            attitude_.pitch = pitch;
        }

        float yaw()
        {
            return attitude_.yaw;
        }

        void yaw(float yaw)
        {
            attitude_.yaw = yaw;
        }

        void updateAttitude(Attitude attitude)
        {
            attitude_.roll = attitude.roll;
            attitude_.pitch = attitude.pitch;
            attitude_.yaw = attitude.yaw;
        }

        void updateAttitude(float roll,  float pitch, float yaw)
        {
            attitude_.roll = roll;
            attitude_.pitch = pitch;
            attitude_.yaw = yaw;
        }

        Attitude attitude()
        {
            return attitude_;
        }

        float linear_velocity_x()
        {
            return speed_.linear_velocity_x;
        }

        void linear_velocity_x(float linear_velocity_x)
        {
            speed_.linear_velocity_x = linear_velocity_x;
        }

        float linear_velocity_y()
        {
            return speed_.linear_velocity_y;
        }

        void linear_velocity_y(float linear_velocity_y)
        {
            speed_.linear_velocity_y = linear_velocity_y;
        }

        float angular_velocity_z()
        {
            return speed_.angular_velocity_z;
        }

        void angular_velocity_z(float angular_velocity_z)
        {
            speed_.angular_velocity_z = angular_velocity_z;
        }

        void updateSpeed(Velocities speed)
        {
            speed_.linear_velocity_x = speed.linear_velocity_x;
            speed_.linear_velocity_y = speed.linear_velocity_y;
            speed_.angular_velocity_z = speed.angular_velocity_z;
        }

        void updateSpeed(float linear_velocity_x,  float linear_velocity_y, float angular_velocity_z)
        {
            speed_.linear_velocity_x = linear_velocity_x;
            speed_.linear_velocity_y = linear_velocity_y;
            speed_.angular_velocity_z = angular_velocity_z;
        }

        Velocities speed()
        {
            return speed_;
        }

        QuadrupedLeg *legs[4];

        QuadrupedLeg *lf;
        QuadrupedLeg *rf;
        QuadrupedLeg *lh;
        QuadrupedLeg *rh;
};

#endif