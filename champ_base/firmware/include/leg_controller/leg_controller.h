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

#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>
#include <quadruped_base/quadruped_components.h>
#include <leg_controller/trajectory_planner.h>
#include <leg_controller/phase_generator.h>

namespace champ
{
    class LegController
    {
            QuadrupedBase *base_;

            champ::TrajectoryPlanner *trajectory_planners_[4];

            float capVelocities(float velocity, float min_velocity, float max_velocity)
            {
                return ((velocity)<(min_velocity)?(min_velocity):((velocity)>(max_velocity)?(max_velocity):(velocity)));
            }

        public:
            LegController(QuadrupedBase &quadruped_base):
                base_(&quadruped_base),     
                phase_generator(base_->gait_config->stance_duration),
                lf(*base_->lf, base_->gait_config->swing_height, base_->gait_config->stance_depth),
                rf(*base_->rf, base_->gait_config->swing_height, base_->gait_config->stance_depth),
                lh(*base_->lh, base_->gait_config->swing_height, base_->gait_config->stance_depth),
                rh(*base_->rh, base_->gait_config->swing_height, base_->gait_config->stance_depth)
            {
                unsigned int total_legs = 0;
                
                trajectory_planners_[total_legs++] = &lf;
                trajectory_planners_[total_legs++] = &rf;
                trajectory_planners_[total_legs++] = &lh;
                trajectory_planners_[total_legs++] = &rh;
            }

            static void transformLeg(float &step_length, float &rotation, QuadrupedLeg &leg, 
                              float step_x, float step_y, float theta)
            {              
                //translate leg in x and y axis, and rotate in z axix
                //this is to project the new location of the leg's tip           
                geometry::Transformation transformed_stance = leg.zero_stance();    
                transformed_stance.Translate(step_x, step_y, 0.0f);
                transformed_stance.RotateZ(theta);

                //find the distance from prev location to new location of the leg's tip
                float delta_x = transformed_stance.X() - leg.zero_stance().X();
                float delta_y = transformed_stance.Y() - leg.zero_stance().Y();

                //the distance from prev to new location of leg tip must be doubled
                //as this is only half of the trajectory
                step_length = sqrtf(pow(delta_x, 2) + pow(delta_y, 2)) * 2.0f;
                
                //how much the foot trajectory must rotate in Z axis
                rotation = atan2f(delta_y, delta_x);
            }
            
            static float raibertHeuristic (float stance_duration, float target_velocity)
            {
                return (stance_duration / 2.0f) * target_velocity;
            }

            void velocityCommand(geometry::Transformation (&foot_positions)[4], champ::Velocities &req_vel)
            {
                //limit all velocities to user input
                req_vel.linear.x = capVelocities(req_vel.linear.x, -base_->gait_config->max_linear_velocity_x, base_->gait_config->max_linear_velocity_x);
                req_vel.linear.y = capVelocities(req_vel.linear.y, -base_->gait_config->max_linear_velocity_y, base_->gait_config->max_linear_velocity_y);
                req_vel.angular.z = capVelocities(req_vel.angular.z, -base_->gait_config->max_angular_velocity_z, base_->gait_config->max_angular_velocity_z);
                
                float tangential_velocity = req_vel.angular.z * base_->lf->center_to_nominal();
                float velocity =  sqrtf(pow(req_vel.linear.x, 2) + pow(req_vel.linear.y + abs(tangential_velocity), 2));
                
                //calculate optimal distance to hop based
                float step_x = raibertHeuristic(base_->gait_config->stance_duration, req_vel.linear.x);
                float step_y = raibertHeuristic(base_->gait_config->stance_duration, req_vel.linear.y);
                float step_theta = raibertHeuristic(base_->gait_config->stance_duration, tangential_velocity);
                
                //calculate the angle from leg when zero to optimal distance to hop
                float theta = sinf(step_theta / base_->lf->center_to_nominal());

                float step_lengths[4] = {0.0f,0.0f,0.0f,0.0f};
                float trajectory_rotations[4] = {0.0f,0.0f,0.0f,0.0f};    
                float sum_of_steps = 0.0f;

                for(unsigned int i = 0; i < 4; i++)
                {
                    //get the step length and foot trajectory rotation for each leg
                    transformLeg(step_lengths[i], trajectory_rotations[i], *base_->legs[i], step_x, step_y, theta);
                    // get the average calculated step lengths since the gait generator has only one parameter for step length
                    sum_of_steps += step_lengths[i];
                }

                //create a saw tooth signal gen so the trajectory planner knows whether it should swing or stride
                phase_generator.run(velocity, sum_of_steps / 4.0f);

                for(unsigned int i = 0; i < 4; i++)
                {
                    //get the point in the swing/stride equation as a function of the sawtooth signal's magnitude for each leg
                    trajectory_planners_[i]->generate(foot_positions[i], step_lengths[i], trajectory_rotations[i], phase_generator.swing_phase_signal[i], phase_generator.stance_phase_signal[i]);
                }
            }

            champ::PhaseGenerator phase_generator;

            champ::TrajectoryPlanner lf;
            champ::TrajectoryPlanner rf;
            champ::TrajectoryPlanner lh;
            champ::TrajectoryPlanner rh;
    };
}

#endif