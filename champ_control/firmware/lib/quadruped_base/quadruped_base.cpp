#include<quadruped_base.h>

QuadrupedBase::QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg):        
    lf(&lf_leg),
    rf(&rf_leg),
    lh(&lh_leg),
    rh(&rh_leg)
{
    unsigned int total_legs;

    legs[total_legs++] = lf;
    legs[total_legs++] = rf;
    legs[total_legs++] = lh;
    legs[total_legs++] = rh;
}

void QuadrupedBase::joints(float *joints)
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
    return roll_;
}

void QuadrupedBase::roll(float roll)
{
    roll_ = roll;
}

float QuadrupedBase::pitch()
{
    return pitch_;
}

void QuadrupedBase::pitch(float pitch)
{
    pitch_ = pitch;
}

float QuadrupedBase::yaw()
{
    return yaw_;
}

void QuadrupedBase::yaw(float yaw)
{
    yaw_ = yaw;
}

void QuadrupedBase::attitude(float roll, float pitch, float yaw)
{
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
}
