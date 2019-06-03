#ifndef _QUADRUPED_GAIT_H_
#define _QUADRUPED_GAIT_H_

#include<Arduino.h>
#include<Geometry.h>
#include<gait_leg_instance.h>

class QuadrupedGait
{
        GaitLegInstance *foot_stances_[4];

        bool lf_gait_pattern_[4];
        bool rf_gait_pattern_[4];
        bool lh_gait_pattern_[4];
        bool rh_gait_pattern_[4];

    public:
        GaitLegInstance lf;
        GaitLegInstance rf;
        GaitLegInstance lh;
        GaitLegInstance rh;

        QuadrupedGait(int frequency, float max_velocity, float max_displacement);
        void generate(Transformation lf_ref_stance, 
    Transformation rf_ref_stance, Transformation lh_ref_stance, Transformation rh_ref_stance, float target_velocity);
};

#endif