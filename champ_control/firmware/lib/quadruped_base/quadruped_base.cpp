#include <quadruped_base.h>

QuadrupedBase::QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg):        
    total_legs_(0),
    lf(&lf_leg),
    rf(&rf_leg),
    lh(&lh_leg),
    rh(&rh_leg)
{
    addLeg(lf);
    addLeg(rf);
    addLeg(lh);
    addLeg(rh);
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