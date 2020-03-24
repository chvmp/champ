/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef PHASE_GENERATOR_H
#define PHASE_GENERATOR_H

#include <macros/macros.h>

namespace champ
{
    class PhaseGenerator
    {
            unsigned long int last_touchdown_;

            bool has_swung_;

            float stance_duration_;

        public:
            PhaseGenerator(float stance_duration):
                last_touchdown_(time_us()),
                has_swung_(false),
                stance_duration_(stance_duration),
                has_started(false),
                stance_phase_signal{0.0f,0.0f,0.0f,0.0f},
                swing_phase_signal{0.0f,0.0f,0.0f,0.0f}
            {
            }        

            void run(float target_velocity, float step_length)
            {
                unsigned long elapsed_time_ref = 0;
                float swing_phase_period = 0.25f * SECONDS_TO_MICROS;
                float leg_clocks[4] = {0.0f,0.0f,0.0f,0.0f};
                float stance_phase_period =  stance_duration_ * SECONDS_TO_MICROS;
                float stride_period = stance_phase_period + swing_phase_period;

                if(target_velocity == 0.0f)
                {
                    elapsed_time_ref = 0;
                    last_touchdown_ = 0;
                    has_swung_ = false;
                    for(unsigned int i = 0; i < 4; i++)
                    {
                        leg_clocks[i] = 0.0f;
                        stance_phase_signal[i] = 0.0f;
                        swing_phase_signal[i] = 0.0f;  
                    }
                    return;
                }

                if(!has_started)
                {
                    has_started = true;
                    last_touchdown_ = time_us();
                }

                if((time_us() - last_touchdown_) >= stride_period)
                {
                    last_touchdown_ = time_us();
                }

                if(elapsed_time_ref >= stride_period)
                    elapsed_time_ref = stride_period;
                else
                    elapsed_time_ref = time_us() - last_touchdown_;

                leg_clocks[0] = elapsed_time_ref - (0.0f * stride_period);
                leg_clocks[1] = elapsed_time_ref - (0.5f * stride_period);
                leg_clocks[2] = elapsed_time_ref - (0.5f * stride_period);
                leg_clocks[3] = elapsed_time_ref - (0.0f * stride_period);

                for(int i = 0; i < 4; i++)
                {
                    if(leg_clocks[i] > 0 and leg_clocks[i] < stance_phase_period)
                        stance_phase_signal[i] = leg_clocks[i] / stance_phase_period;
                    else
                        stance_phase_signal[i] = 0;

                    if(leg_clocks[i] > -swing_phase_period && leg_clocks[i] < 0)
                        swing_phase_signal[i] = (leg_clocks[i] + swing_phase_period) / swing_phase_period;
                    else if(leg_clocks[i] > stance_phase_period && leg_clocks[i] < stride_period)
                        swing_phase_signal[i] = (leg_clocks[i] - stance_phase_period) / swing_phase_period;
                    else
                        swing_phase_signal[i] = 0;
                }

                if(!has_swung_ && stance_phase_signal[0] < 0.5)
                {
                    stance_phase_signal[0] = 0.0;
                    stance_phase_signal[3] = 0.0;
                    swing_phase_signal[1] = 0.0;
                    swing_phase_signal[2] = 0.0;
                }
                else
                {
                    has_swung_ = true;
                }  
            }

            bool has_started;

            float stance_phase_signal[4];
            float swing_phase_signal[4];
    };
}

#endif