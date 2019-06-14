#include<phase_generator.h>

PhaseGenerator::PhaseGenerator(QuadrupedLeg *leg, int frequency, float max_velocity, float max_displacement):
    leg_(leg),
    frequency_(frequency),
    max_velocity_(max_velocity),
    max_displacement_(max_displacement),
    ref_has_touchdown_(false),
    last_touchdown_(0),
    phase_gen_started_(false),
    leg_clocks_{0,0,0,0},
    stance_phase_signal{0,0,0,0},
    swing_phase_signal{0,0,0,0}
{
}

void PhaseGenerator::startPhaseGen()
{
    if(!phase_gen_started_)
        last_touchdown_ = millis();
}

void PhaseGenerator::stopPhaseGen()
{

}

void PhaseGenerator::run(float target_velocity)
{
    unsigned long now = millis();
    float stance_phase_period =  (max_displacement_ / target_velocity);
    float swing_phase_period = 2500;
    float stride_period = stance_phase_period + swing_phase_period;
    static float TD = swing_phase_period;
    // if(!ref_has_touchdown_ && !phase_gen_started_)
    // {
    //     TD = 2500;
    //             last_touchdown_ = now;

    // }

    // if(!phase_gen_started_)
    // {
    //     last_touchdown_ = now;
    //     phase_gen_started_ = true;
    // }


    //this is open loop for now;
    // if(stance_phase_signal[0] >= 0.8)
    //     last_touchdown_ = now;


    if(now - last_touchdown_>= TD)
    {
        if(!ref_has_touchdown_)
        {
            ref_has_touchdown_ = true;
            TD = stride_period;
        }

        last_touchdown_ = now;
    }   
    // else 
    // {
    //     ref_has_touchdown_ = false;
    // }
    // if(now - last_touchdown_ >= TD)
    // {
    //     last_touchdown_ = now;
    //     TD = 10000;
    //     ref_has_touchdown_ = true;
    //     phase_gen_started_ = true;
    // }


    unsigned long int elapsed_time_ref = now - last_touchdown_;
    elapsed_time_ref = constrain(elapsed_time_ref, 0, stride_period);

    leg_clocks_[0] = elapsed_time_ref - (0 * stride_period);
    leg_clocks_[1] = elapsed_time_ref - (0.5 * stride_period);
    leg_clocks_[2] = elapsed_time_ref - (0.5 * stride_period);
    leg_clocks_[3] = elapsed_time_ref - (0 * stride_period);
  

    for(int i = 0; i < 4; i++)
    {
        stance_phase_signal[i] = leg_clocks_[i] / stance_phase_period;
        stance_phase_signal[i] = constrain(stance_phase_signal[i], 0 , stance_phase_period);
    }

    for(int i = 0; i < 4; i++)
    {
        float temp_t = 0;

        if(leg_clocks_[i] > -swing_phase_period && leg_clocks_[i] < 0)
            temp_t = swing_phase_period;
        else  if(leg_clocks_[i] > stance_phase_period && leg_clocks_[i] < stride_period)
            temp_t = -stance_phase_period;

        swing_phase_signal[i] = (leg_clocks_[i] + temp_t) / swing_phase_period;
    }


    // swing_phase_signal[0]
    // swing_phase_signal[1]
    // swing_phase_signal[2]
    // swing_phase_signal[3]
}

