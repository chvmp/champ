#include<quadruped_gait.h>

QuadrupedGait::QuadrupedGait(QuadrupedBase &quadruped_base, int frequency, float max_velocity, float max_displacement):
    base_(&quadruped_base),
    lf_gait_pattern_{1,0,1,0},
    rf_gait_pattern_{0,1,0,1},
    lh_gait_pattern_{0,1,0,1},
    rh_gait_pattern_{1,0,1,0},
    lf(base_->lf, frequency, max_velocity, max_displacement),
    rf(base_->rf, frequency, max_velocity, max_displacement),
    lh(base_->lh, frequency, max_velocity, max_displacement),
    rh(base_->rh, frequency, max_velocity, max_displacement),
    phase_gen_(base_->rh, frequency, max_velocity, max_displacement)
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
    lf.generate(lf_ref_stance, target_velocity, lf_gait_pattern_, phase_gen_.swing_phase_signal[0], phase_gen_.stance_phase_signal[0]);
    rf.generate(rf_ref_stance, target_velocity, rf_gait_pattern_, phase_gen_.swing_phase_signal[1], phase_gen_.stance_phase_signal[1]);
    lh.generate(lh_ref_stance, target_velocity, lh_gait_pattern_, phase_gen_.swing_phase_signal[2], phase_gen_.stance_phase_signal[2]);
    rh.generate(rh_ref_stance, target_velocity, rh_gait_pattern_, phase_gen_.swing_phase_signal[3], phase_gen_.stance_phase_signal[3]);

    // base_->lf->foot_base_to_hip(lf.foot_);
    // base_->rf->foot_base_to_hip(rf.foot_);
    // base_->lh->foot_base_to_hip(lh.foot_);
    // base_->rh->foot_base_to_hip(rh.foot_);
}