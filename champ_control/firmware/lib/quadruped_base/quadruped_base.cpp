#include<quadruped_base.h>

QuadrupedBase::QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg):        
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
        legs[i]->leg_id(i);
    }
}

void QuadrupedBase::getJointStates(float *joints)
{
    unsigned int total_joints = 0;

    for(unsigned int i=0; i < 4; i++)
    {
        joints[total_joints++] = legs[i]->hip->theta();
        joints[total_joints++] = legs[i]->upper_leg->theta();
        joints[total_joints++] = legs[i]->lower_leg->theta();
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
