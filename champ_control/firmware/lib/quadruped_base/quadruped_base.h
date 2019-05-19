#include <quadruped_leg.h>

class QuadrupedBase
{   
    unsigned int total_legs_;
    unsigned int total_joints_;
    
    float *joints_[12];

    void addLeg(QuadrupedLeg *leg)
    {
        legs[total_legs_++] = leg;
          
        joints_[total_joints_++] = &legs[total_legs_]->hip();
        joints_[total_joints_++] = &legs[total_legs_]->upper_leg();
        joints_[total_joints_++] = &legs[total_legs_]->lower_leg();
    }
    
    public:

        QuadrupedLeg *legs[4];

        QuadrupedLeg *lf;
        QuadrupedLeg *rf;
        QuadrupedLeg *lh;
        QuadrupedLeg *rh;

        QuadrupedBase(QuadrupedLeg &lf_leg, QuadrupedLeg &rf_leg, QuadrupedLeg &lh_leg, QuadrupedLeg &rh_leg):        
            total_legs_(0)
        {
            lf = &lf_leg;
            rf = &rf_leg;
            lh = &lh_leg;
            rh = &rh_leg;

            addLeg(lf);
            addLeg(rf);
            addLeg(lh);
            addLeg(rh);
        }

        float *joints()
        {
            return *joints_;
        }
};


