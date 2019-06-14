#ifndef _QUADRUPED_GAIT_H_
#define _QUADRUPED_GAIT_H_

#include<Arduino.h>
#include<Geometry.h>
#include<quadruped_base.h>
#include<gait_leg_instance.h>
#include<phase_generator.h>

class QuadrupedGait
{
        QuadrupedBase *base_;

        GaitLegInstance *foot_stances_[4];

        bool lf_gait_pattern_[4];
        bool rf_gait_pattern_[4];
        bool lh_gait_pattern_[4];
        bool rh_gait_pattern_[4];

        PhaseGenerator phase_gen_;
    public:
        GaitLegInstance lf;
        GaitLegInstance rf;
        GaitLegInstance lh;
        GaitLegInstance rh;

        QuadrupedGait(QuadrupedBase &quadruped_base, int frequency, float max_velocity, float max_displacement);
        void generate(Transformation lf_ref_stance, 
        Transformation rf_ref_stance, Transformation lh_ref_stance, Transformation rh_ref_stance, float target_velocity);
};

#endif