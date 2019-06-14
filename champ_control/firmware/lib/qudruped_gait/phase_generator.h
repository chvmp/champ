#ifndef _PHASE_GENERATOR_H_
#define _PHASE_GENERATOR_H_

#include<Arduino.h>
#include<Geometry.h>
#include<quadruped_leg.h>

class PhaseGenerator
{
        QuadrupedLeg *leg_;
        int frequency_;
        float max_velocity_;
        float max_displacement_;

        bool ref_has_touchdown_;
        unsigned long int last_touchdown_;
        bool phase_gen_started_;

        float leg_clocks_[4];

    public:
        float stance_phase_signal[4];
        float swing_phase_signal[4];
        PhaseGenerator(QuadrupedLeg *leg, int frequency, float max_velocity, float max_displacement);
        void startPhaseGen();
        void stopPhaseGen();
        void run(float target_velocity);
};

#endif