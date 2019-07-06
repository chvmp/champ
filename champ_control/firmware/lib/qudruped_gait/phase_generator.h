#ifndef _PHASE_GENERATOR_H_
#define _PHASE_GENERATOR_H_

#include<Arduino.h>
#include<Geometry.h>
#include<quadruped_leg.h>
#include <string.h>

class PhaseGenerator
{
        QuadrupedLeg *leg_;

        unsigned long int last_touchdown_;
        bool phase_gen_started_;

        float leg_clocks_[4];

    public:
        PhaseGenerator();

        float stance_phase_signal[4];
        float swing_phase_signal[4];
        
        void run(float step_distance, float target_velocity);
};

#endif