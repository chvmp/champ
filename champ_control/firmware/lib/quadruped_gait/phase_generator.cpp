#include<phase_generator.h>
PhaseGenerator::PhaseGenerator(float step_length):
    step_length_(step_length),
    last_touchdown_(millis()),
    leg_clocks_{0,0,0,0},
    has_swung_(false),
    has_started(false),
    stance_phase_signal{0,0,0,0},
    swing_phase_signal{0,0,0,0}
{
}

void PhaseGenerator::run(float target_velocity)
{
    unsigned long elapsed_time_ref = 0;
    float swing_phase_period = 250;
    float leg_clocks[4] = {0,0,0,0};

    if(target_velocity == 0)
    {
        elapsed_time_ref = 0;
        last_touchdown_ = 0;
        has_swung_ = false;
        for(unsigned int i = 0; i < 4; i++)
        {
            leg_clocks_[i] = 0;
            stance_phase_signal[i] = 0;
            swing_phase_signal[i] = 0;  
        }
        return;
    }

    if(!has_started)
    {
        has_started = true;
        last_touchdown_ = millis();
    }

    float stance_phase_period =  (step_length_ / target_velocity) * 1000;
    float stride_period = stance_phase_period + swing_phase_period;

    if((millis() - last_touchdown_) >= stride_period)
    {
        last_touchdown_ = millis();
    }

    if(elapsed_time_ref >= stride_period)
        elapsed_time_ref = stride_period;
    else
        elapsed_time_ref = millis() - last_touchdown_;

    leg_clocks[0] = elapsed_time_ref - (0 * stride_period);
    leg_clocks[1] = elapsed_time_ref - (0.5 * stride_period);
    leg_clocks[2] = elapsed_time_ref - (0.5 * stride_period);
    leg_clocks[3] = elapsed_time_ref - (0 * stride_period);

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

    if(swing_phase_signal[0] > 0)
        has_swung_ = true;
    
    if(!has_swung_)
    {
        if(stance_phase_signal[0] < 0.5)
            stance_phase_signal[0] = 0.5;
            stance_phase_signal[3] = 0.5; 
    }  
}
    
