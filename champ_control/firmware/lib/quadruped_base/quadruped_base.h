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

        QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg);
        void joints(float *joints);
};

#endif


