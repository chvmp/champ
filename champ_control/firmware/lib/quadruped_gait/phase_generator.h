#ifndef _PHASE_GENERATOR_H_
#define _PHASE_GENERATOR_H_

#include<Arduino.h>
#include<Geometry.h>
#include<quadruped_leg.h>
#include<champ_config.h>

class PhaseGenerator
{
        float step_length_;
        unsigned long int last_touchdown_;

        float leg_clocks_[4];

        bool has_swung_;

    public:
        PhaseGenerator(float step_length);
        
        bool has_started;

        float stance_phase_signal[4];
        float swing_phase_signal[4];
        
        void run(float target_velocity);
};

#endif