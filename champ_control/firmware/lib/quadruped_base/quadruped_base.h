#ifndef _QUADRUPPED_BASE_H_
#define _QUADRUPPED_BASE_H_

#include<quadruped_leg.h>

class QuadrupedBase
{   
    unsigned int total_legs_;
    float roll_;
    float pitch_;
    float yaw_;

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

        float roll();
        void roll(float roll);

        float pitch();
        void pitch(float pitch);
        
        float yaw();
        void yaw(float yaw);

        void attitude(float roll, float pitch, float yaw);
};

#endif


