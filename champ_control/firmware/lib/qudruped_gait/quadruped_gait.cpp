#include<quadruped_gait.h>

QuadrupedGait::QuadrupedGait(QuadrupedBase &quadruped_base, float max_velocity, float swing_height, float stance_depth, float step_length):
    base_(&quadruped_base),
    foot_stances_{0,0,0,0},
    phase_gen_(),
    lf(swing_height, stance_depth),
    rf(swing_height, stance_depth),
    lh(swing_height, stance_depth),
    rh(swing_height, stance_depth)
{
    unsigned int total_stances = 0;
    
    foot_stances_[total_stances++] = &lf;
    foot_stances_[total_stances++] = &rf;
    foot_stances_[total_stances++] = &lh;
    foot_stances_[total_stances++] = &rh;
}

void QuadrupedGait::generate(Transformation lf_ref_stance, Transformation rf_ref_stance, Transformation lh_ref_stance, Transformation rh_ref_stance, 
float linear_velocity_x, float linear_velocity_y, float angular_velocity_z)
{
    float theta = 1.5708;

    phase_gen_.run(linear_velocity_x, 0.0675);
    // lf.generate(lf_ref_stance, linear_velocity_x, 1.5708 + theta, phase_gen_.swing_phase_signal[0], phase_gen_.stance_phase_signal[0]);
    // rf.generate(rf_ref_stance, linear_velocity_x, theta, phase_gen_.swing_phase_signal[1], phase_gen_.stance_phase_signal[1]);
    // lh.generate(lh_ref_stance, linear_velocity_x, -1.5708 - theta, phase_gen_.swing_phase_signal[2], phase_gen_.stance_phase_signal[2]);
    // rh.generate(rh_ref_stance, linear_velocity_x, -theta, phase_gen_.swing_phase_signal[3], phase_gen_.stance_phase_signal[3]);
    // lf.generate(lf_ref_stance, linear_velocity_x, -theta, phase_gen_.swing_phase_signal[0], phase_gen_.stance_phase_signal[0]);
    // rf.generate(rf_ref_stance, linear_velocity_x, -1.5708 - theta, phase_gen_.swing_phase_signal[1], phase_gen_.stance_phase_signal[1]);
    // lh.generate(lh_ref_stance, linear_velocity_x, theta, phase_gen_.swing_phase_signal[2], phase_gen_.stance_phase_signal[2]);
    // rh.generate(rh_ref_stance, linear_velocity_x, 1.5708 + theta, phase_gen_.swing_phase_signal[3], phase_gen_.stance_phase_signal[3]);
    // lf.generate(lf_ref_stance, 0 - theta, phase_gen_.swing_phase_signal[0], phase_gen_.stance_phase_signal[0]);
    // rf.generate(rf_ref_stance, PI + theta, phase_gen_.swing_phase_signal[1], phase_gen_.stance_phase_signal[1]);
    // lh.generate(lh_ref_stance, 0 + theta, phase_gen_.swing_phase_signal[2], phase_gen_.stance_phase_signal[2]);
    // rh.generate(rh_ref_stance, PI - theta, phase_gen_.swing_phase_signal[3], phase_gen_.stance_phase_signal[3]);
    lf.generate(lf_ref_stance, 0, 0.09, phase_gen_.swing_phase_signal[0], phase_gen_.stance_phase_signal[0]);
    rf.generate(rf_ref_stance, 0, 0.09 * 0.5, phase_gen_.swing_phase_signal[1], phase_gen_.stance_phase_signal[1]);
    lh.generate(lh_ref_stance, 0, 0.09, phase_gen_.swing_phase_signal[2], phase_gen_.stance_phase_signal[2]);
    rh.generate(rh_ref_stance, 0, 0.09 * 0.5, phase_gen_.swing_phase_signal[3], phase_gen_.stance_phase_signal[3]);
}