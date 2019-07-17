#ifndef _QUADRUPPED_BASE_H_
#define _QUADRUPPED_BASE_H_

#include<quadruped_leg.h>

class QuadrupedBase
{   
    float current_roll_;
    float current_pitch_;
    float current_yaw_;
    
    public:
        QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg);
        void joints(float *joints);

        float roll();
        void roll(float roll);

        float pitch();
        void pitch(float pitch);

        float yaw();
        void yaw(float yaw);

        void attitude(float roll, float pitch, float yaw);

        QuadrupedLeg *legs[4];

        QuadrupedLeg *lf;
        QuadrupedLeg *rf;
        QuadrupedLeg *lh;
        QuadrupedLeg *rh;
};

#endif


