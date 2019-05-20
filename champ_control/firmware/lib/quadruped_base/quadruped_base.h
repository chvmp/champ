#ifndef _QUADRUPPED_BASE_H_
#define  _QUADRUPPED_BASE_H_

#include <quadruped_leg.h>

class QuadrupedBase
{   
    unsigned int total_legs_;
    
    void addLeg(QuadrupedLeg *leg)
    {
        legs[total_legs_++] = leg;
    }
    
    public:

        QuadrupedLeg *legs[4];

        QuadrupedLeg *lf;
        QuadrupedLeg *rf;
        QuadrupedLeg *lh;
        QuadrupedLeg *rh;

        QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg):        
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

        void joints(float *joints)
        {
            unsigned int total_joints = 0;

            for(int i=0; i < 4; i++)
            {
                joints[total_joints++] = legs[total_legs_]->hip();
                joints[total_joints++] = legs[total_legs_]->upper_leg();
                joints[total_joints++] = legs[total_legs_]->lower_leg();
            }
        }
};

#endif


