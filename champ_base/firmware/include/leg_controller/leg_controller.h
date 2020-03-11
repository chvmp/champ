#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>
#include <leg_controller/trajectory_planner.h>
#include <leg_controller/phase_generator.h>

namespace champ
{
    class LegController
    {
            QuadrupedBase *base_;

            champ::TrajectoryPlanner *trajectory_planners_[4];

            champ::PhaseGenerator phase_gen_;
            float max_linear_velocity_x_;
            float max_linear_velocity_y_;
            float max_angular_velocity_;
            float stance_duration_;
            float max_theta_;

            float capVelocities(float velocity, float min_velocity, float max_velocity)
            {
                return ((velocity)<(min_velocity)?(min_velocity):((velocity)>(max_velocity)?(max_velocity):(velocity)));
            }

        public:
            LegController(QuadrupedBase &quadruped_base, float max_linear_velocity_x, float max_linear_velocity_y, float max_angular_velocity, float stance_duration, float swing_height, float stance_depth):
                base_(&quadruped_base),
                trajectory_planners_{0,0,0,0},
                phase_gen_(stance_duration),
                max_linear_velocity_x_(max_linear_velocity_x),
                max_linear_velocity_y_(max_linear_velocity_y),
                max_angular_velocity_(max_angular_velocity),
                stance_duration_(stance_duration),
                lf(*base_->lf, swing_height, stance_depth),
                rf(*base_->rf, swing_height, stance_depth),
                lh(*base_->lh, swing_height, stance_depth),
                rh(*base_->rh, swing_height, stance_depth)
            {
                unsigned int total_legs = 0;
                
                trajectory_planners_[total_legs++] = &lf;
                trajectory_planners_[total_legs++] = &rf;
                trajectory_planners_[total_legs++] = &lh;
                trajectory_planners_[total_legs++] = &rh;
            }

            void transformLeg(float &step_length, float &rotation, QuadrupedLeg &leg, float linear_velocity_x, float linear_velocity_y, float angular_velocity_z)
            {   
                float tangential_velocity = angular_velocity_z * leg.center_to_nominal();

                float step_x = raibertHeuristic(stance_duration_, linear_velocity_x);
                float step_y = raibertHeuristic(stance_duration_, linear_velocity_y);
                float step_theta = raibertHeuristic(stance_duration_, tangential_velocity);
                                                        
                float center_to_nominal = sqrtf(pow(base_->lf->zero_stance().X(), 2) + pow(base_->lf->zero_stance().Y(), 2));
                float theta = sinf(step_theta / center_to_nominal);

                Transformation transformed_stance = leg.zero_stance();    
                transformed_stance.Translate(step_x, step_y, 0);
                transformed_stance.RotateZ(theta);

                float delta_x = transformed_stance.X() - leg.zero_stance().X();
                float delta_y = transformed_stance.Y() - leg.zero_stance().Y();

                step_length = sqrtf(pow(delta_x, 2) + pow(delta_y, 2)) * 2;
                rotation = atan2f(delta_y, delta_x);
            }
            
            float raibertHeuristic(float stance_duration, float target_velocity)
            {
                return (stance_duration / 2) * target_velocity;
            }

            void velocityCommand(Transformation (&foot_positions)[4], float linear_velocity_x, float linear_velocity_y, float angular_velocity_z)
            {
                linear_velocity_x = capVelocities(linear_velocity_x, -max_linear_velocity_x_, max_linear_velocity_x_);
                linear_velocity_y = capVelocities(linear_velocity_y, -max_linear_velocity_y_, max_linear_velocity_y_);
                angular_velocity_z = capVelocities(angular_velocity_z, -max_angular_velocity_, max_angular_velocity_);
                
                float tangential_velocity = angular_velocity_z * base_->lf->center_to_nominal();
                float velocity =  sqrtf(pow(linear_velocity_x, 2) + pow(linear_velocity_y + abs(tangential_velocity), 2));
                float step_lengths[4] = {0,0,0,0};
                float trajectory_rotations[4] = {0,0,0,0};    
                float sum_of_steps = 0;

                for(unsigned int i = 0; i < 4; i++)
                {
                    //get the step length and foot trajectory rotation for each leg
                    transformLeg(step_lengths[i], trajectory_rotations[i], *base_->legs[i], linear_velocity_x, linear_velocity_y, angular_velocity_z);
                    // get the average calculated step lengths since the gait generator has only one parameter for step length
                    sum_of_steps += step_lengths[i];
                }

                //create a saw tooth signal gen so the trajectory planner knows whether it should swing or stride
                phase_gen_.run(velocity, sum_of_steps / 4.0);

                for(unsigned int i = 0; i < 4; i++)
                {
                    //get the point in the swing/stride equation as a function of the sawtooth signal's magnitude for each leg
                    trajectory_planners_[i]->generate(foot_positions[i], step_lengths[i], trajectory_rotations[i], phase_gen_.swing_phase_signal[i], phase_gen_.stance_phase_signal[i]);
                }
            }

            champ::TrajectoryPlanner lf;
            champ::TrajectoryPlanner rf;
            champ::TrajectoryPlanner lh;
            champ::TrajectoryPlanner rh;
    };
}

#endif