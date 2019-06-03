#include<quadruped_gait.h>

QuadrupedGait::QuadrupedGait(int frequency, float max_velocity, float max_displacement):
    lf_gait_pattern_{1,0,1,0},
    rf_gait_pattern_{0,1,0,1},
    lh_gait_pattern_{0,1,0,1},
    rh_gait_pattern_{1,0,1,0},
    lf(frequency, max_velocity, max_displacement),
    rf(frequency, max_velocity, max_displacement),
    lh(frequency, max_velocity, max_displacement),
    rh(frequency, max_velocity, max_displacement)
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
    lf.generate(lf_ref_stance, target_velocity, lf_gait_pattern_);
    rf.generate(rf_ref_stance, target_velocity, rf_gait_pattern_);
    lh.generate(lh_ref_stance, target_velocity, lh_gait_pattern_);
    rh.generate(rh_ref_stance, target_velocity, rh_gait_pattern_);
}