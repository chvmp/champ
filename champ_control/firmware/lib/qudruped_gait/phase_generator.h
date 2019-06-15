#ifndef _PHASE_GENERATOR_H_
#define _PHASE_GENERATOR_H_

#include<Arduino.h>
#include<Geometry.h>
#include<quadruped_leg.h>
#include <string.h>

class PhaseGenerator
{
        QuadrupedLeg *leg_;
        float max_velocity_;
        float max_displacement_;

        unsigned long int last_touchdown_;
        bool phase_gen_started_;

        float leg_clocks_[4];

    public:
        PhaseGenerator(QuadrupedLeg *leg, float max_velocity, float max_displacement);

        float stance_phase_signal[4];
        float swing_phase_signal[4];
        
        void run(float target_velocity);
};

#endif