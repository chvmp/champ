#include<quadruped_gait.h>

QuadrupedGait::QuadrupedGait(QuadrupedBase &quadruped_base, float max_velocity, float swing_height, float step_length, float stance_depth):
    base_(&quadruped_base),
    foot_stances_{0,0,0,0},
    phase_gen_(step_length),
    lf(base_->lf, swing_height, step_length, stance_depth),
    rf(base_->rf, swing_height, step_length, stance_depth),
    lh(base_->lh, swing_height, step_length, stance_depth),
    rh(base_->rh, swing_height, step_length, stance_depth)
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
    float tangential_velocity = angular_velocity_z * (base_->lf->nominal_stance().Y() +  base_->lf->nominal_stance().X());
    float velocity =  sqrt(pow(linear_velocity_x + tangential_velocity, 2) + pow(linear_velocity_y + tangential_velocity, 2));

    phase_gen_.run(velocity);

    lf.generate(lf_ref_stance, linear_velocity_x, linear_velocity_y, angular_velocity_z, phase_gen_.swing_phase_signal[0], phase_gen_.stance_phase_signal[0]);
    rf.generate(rf_ref_stance, linear_velocity_x, linear_velocity_y, angular_velocity_z, phase_gen_.swing_phase_signal[1], phase_gen_.stance_phase_signal[1]);
    lh.generate(lh_ref_stance, linear_velocity_x, linear_velocity_y, angular_velocity_z, phase_gen_.swing_phase_signal[2], phase_gen_.stance_phase_signal[2]);
    rh.generate(rh_ref_stance, linear_velocity_x, linear_velocity_y, angular_velocity_z, phase_gen_.swing_phase_signal[3], phase_gen_.stance_phase_signal[3]);
}