#include<quadruped_gait.h>

QuadrupedGait::QuadrupedGait(QuadrupedBase &quadruped_base, float max_velocity, float swing_height, float max_displacement):
    base_(&quadruped_base),
    foot_stances_{0,0,0,0},
    phase_gen_(base_->rh, max_velocity, max_displacement),
    lf(base_->lf, max_velocity, swing_height, max_displacement),
    rf(base_->rf, max_velocity, swing_height, max_displacement),
    lh(base_->lh, max_velocity, swing_height, max_displacement),
    rh(base_->rh, max_velocity, swing_height, max_displacement)
{
    unsigned int total_stances = 0;
    
    foot_stances_[total_stances++] = &lf;
    foot_stances_[total_stances++] = &rf;
    foot_stances_[total_stances++] = &lh;
    foot_stances_[total_stances++] = &rh;
}

void QuadrupedGait::generate(Transformation lf_ref_stance, 
    Transformation rf_ref_stance, Transformation lh_ref_stance, Transformation rh_ref_stance, float target_velocity)
{
    phase_gen_.run(target_velocity);
    lf.generate(lf_ref_stance, target_velocity, 1.57, phase_gen_.swing_phase_signal[0], phase_gen_.stance_phase_signal[0]);
    rf.generate(rf_ref_stance, target_velocity, 1.57, phase_gen_.swing_phase_signal[1], phase_gen_.stance_phase_signal[1]);
    lh.generate(lh_ref_stance, target_velocity, 1.57, phase_gen_.swing_phase_signal[2], phase_gen_.stance_phase_signal[2]);
    rh.generate(rh_ref_stance, target_velocity, 1.57, phase_gen_.swing_phase_signal[3], phase_gen_.stance_phase_signal[3]);
}