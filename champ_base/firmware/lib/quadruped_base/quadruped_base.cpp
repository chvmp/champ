#include<quadruped_base.h>

QuadrupedBase::QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg, const char *knee_orientation):        
    knee_orientation_(knee_orientation),
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
            dir = getKneeDirection(knee_orientation_[0]);
        }
        else
        {
            dir = getKneeDirection(knee_orientation_[1]);
        }
        
        legs[i]->knee_direction(dir);
    }
}

int QuadrupedBase::getKneeDirection(char direction)
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

void QuadrupedBase::getJointStates(float *joints)
{
    unsigned int total_joints = 0;

    for(unsigned int i = 0; i < 4; i++)
    {
        joints[total_joints++] = legs[i]->hip->theta();
        joints[total_joints++] = legs[i]->upper_leg->theta();
        joints[total_joints++] = legs[i]->lower_leg->theta();
    }
}

void QuadrupedBase::joint_states(float joints[12])
{
    for(unsigned int i = 0; i < 4; i++)
    {
        int index = i * 3;
        legs[i]->hip->theta(joints[index]);
        legs[i]->upper_leg->theta(joints[index + 1]);
        legs[i]->lower_leg->theta(joints[index + 2]);
    }
}

float QuadrupedBase::roll()
{
    return current_roll_;
}

void QuadrupedBase::roll(float roll)
{
    current_roll_ = roll;
}

float QuadrupedBase::pitch()
{
    return current_pitch_;
}

void QuadrupedBase::pitch(float pitch)
{
    current_pitch_ = pitch;
}

float QuadrupedBase::yaw()
{
    return current_yaw_;
}

void QuadrupedBase::yaw(float yaw)
{
    current_yaw_ = yaw;
}

void QuadrupedBase::attitude(float roll, float pitch, float yaw)
{
    current_roll_ = roll;
    current_pitch_ = pitch;
    current_yaw_ = yaw;
}
