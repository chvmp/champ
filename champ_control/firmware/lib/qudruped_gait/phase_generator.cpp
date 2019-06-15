#include<phase_generator.h>
PhaseGenerator::PhaseGenerator(QuadrupedLeg *leg, float max_velocity, float max_displacement):
    leg_(leg),
    max_velocity_(max_velocity),
    max_displacement_(max_displacement),
    last_touchdown_(0),
    phase_gen_started_(false),
    leg_clocks_{0,0,0,0},
    stance_phase_signal{0,0,0,0},
    swing_phase_signal{0,0,0,0}
{
}

void PhaseGenerator::run(float target_velocity)
{
    unsigned long now = millis();
    float stance_phase_period =  (max_displacement_ / target_velocity) * 1000;
    float swing_phase_period = 250;
    float stride_period = stance_phase_period + swing_phase_period;
    float temp_stance_phase_signal[4] = {0,0,0,0};
    float temp_swing_phase_signal[4] = {0,0,0,0};
    float leg_clocks[4] = {0,0,0,0};

    if(!phase_gen_started_)
    {
        phase_gen_started_ = true;
        last_touchdown_ = now;
    }

    if((now - last_touchdown_) > stride_period)
        last_touchdown_ = now;
  
    unsigned long int elapsed_time_ref = now - last_touchdown_;
    if(elapsed_time_ref > stride_period)
        elapsed_time_ref = stride_period;

    leg_clocks[0] = elapsed_time_ref - (0.0 * stride_period);
    leg_clocks[1] = elapsed_time_ref - (0.5 * stride_period);
    leg_clocks[2] = elapsed_time_ref - (0.5 * stride_period);
    leg_clocks[3] = elapsed_time_ref - (0.0 * stride_period);

    for(int i = 0; i < 4; i++)
    {
        temp_stance_phase_signal[i] = leg_clocks[i] / stance_phase_period;
        temp_stance_phase_signal[i] = constrain(temp_stance_phase_signal[i], 0, stance_phase_period);

        if(leg_clocks[i] > -swing_phase_period && leg_clocks[i] < 0)
            temp_swing_phase_signal[i] = (leg_clocks[i] + swing_phase_period) / swing_phase_period;

        else  if(leg_clocks[i] > stance_phase_period && leg_clocks[i] < stride_period)
            temp_swing_phase_signal[i] = (leg_clocks[i] - stance_phase_period) / swing_phase_period;

        if(temp_stance_phase_signal[i] > 1)
            temp_stance_phase_signal[i] = 0;

        if(temp_swing_phase_signal[i] > 1)
            temp_swing_phase_signal[i] = 0;
    }
    
    for(int i = 0; i < 4; i++)
    {
        stance_phase_signal[i] = temp_stance_phase_signal[i];
        swing_phase_signal[i] = temp_swing_phase_signal[i];
    }
}
