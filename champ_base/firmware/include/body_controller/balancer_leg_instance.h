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

#ifndef BALANCER_LEG_INSTANCE
#define BALANCER_LEG_INSTANCE

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_leg.h>

namespace champ
{
    class BalancerLegInstance
    {
        QuadrupedLeg *leg_;
        
        public:
            BalancerLegInstance(QuadrupedLeg &leg):
                leg_(&leg)
            {
            }

            void balance(geometry::Transformation &foot_position, float body_roll, float body_pitch, 
                                    float body_yaw, float target_z)
            {
                geometry::Transformation normal_vector;
                geometry::Transformation plane_p;
                geometry::Transformation normal_vector_origin;
                geometry::Transformation line_p0;
                geometry::Transformation line_p1;

                geometry::Point line_vector;
                float d;
                float delta_height = leg_->zero_stance().Z() + target_z;

                plane_p = leg_->zero_stance();
                plane_p.Translate(0.0f, 0.0f, -delta_height);

                line_p0 = leg_->zero_stance();
                line_p1 = leg_->zero_stance();
                line_p0.Translate(0.0f, 0.0f, -delta_height);
                line_p1.Translate(0.0f, 0.0f, leg_->lower_leg->x() + leg_->foot->x());

                normal_vector_origin = leg_->zero_stance();
                normal_vector_origin.Translate(-0.1, -0.1, -delta_height);
                normal_vector.p = normal_vector_origin.p;
                normal_vector.Translate(0.0f, 0.0f, 0.1);

                plane_p.RotateX(body_roll);
                plane_p.RotateY(body_pitch);

                line_p0.RotateZ(body_yaw);

                normal_vector_origin.RotateX(body_roll);
                normal_vector_origin.RotateY(body_pitch);

                normal_vector.RotateX(body_roll);
                normal_vector.RotateY(body_pitch);

                normal_vector.X() = normal_vector.X() - normal_vector_origin.X();
                normal_vector.Y() = normal_vector.Y() - normal_vector_origin.Y();
                normal_vector.Z() = normal_vector.Z() - normal_vector_origin.Z();

                line_vector.X() = -(line_p1.X() - line_p0.X());
                line_vector.Y() = -(line_p1.Y() - line_p0.Y());
                line_vector.Z() = -(line_p1.Z() - line_p0.Z());

                d = (plane_p.X() * normal_vector.p.X() + plane_p.Y() * normal_vector.p.Y() +  plane_p.Z() * normal_vector.p.Z());

                BLA::Matrix<4,4> denominator = 
                {
                    normal_vector.X(), normal_vector.Y(),   normal_vector.Z(),               0,
                                    1,                 0,                   0, line_vector.X(),
                                    0,                 1,                   0, line_vector.Y(),
                                    0,                 0,                   1, line_vector.Z()
                };

                BLA::Matrix<4,4> x_numerator = 
                {
                            d, normal_vector.Y(),   normal_vector.Z(),               0,
                    line_p0.X(),                 0,                   0, line_vector.X(),
                    line_p0.Y(),                 1,                   0, line_vector.Y(),
                    line_p0.Z(),                 0,                   1, line_vector.Z()
                };

                BLA::Matrix<4,4> y_numerator = 
                {
                    normal_vector.X(),           d,   normal_vector.Z(),               0,
                                    1, line_p0.X(),                   0, line_vector.X(),
                                    0, line_p0.Y(),                   0, line_vector.Y(),
                                    0, line_p0.Z(),                   1, line_vector.Z()
                };

                BLA::Matrix<4,4> z_numerator = 
                {
                    normal_vector.X(), normal_vector.Y(),           d,               0,
                                    1,                 0, line_p0.X(), line_vector.X(),
                                    0,                 1, line_p0.Y(), line_vector.Y(),
                                    0,                 0, line_p0.Z(), line_vector.Z()
                };

                foot_position.p.X() = x_numerator.Det() / denominator.Det();
                foot_position.p.Y() = y_numerator.Det() / denominator.Det();
                foot_position.p.Z() = z_numerator.Det() / denominator.Det();
                
                leg_->transformToHip(foot_position);
            }

            void poseCommand(geometry::Transformation &foot_position, champ::Pose &req_pose)
            {
                float delta_height = leg_->zero_stance().Z() + req_pose.translation.z;

                //create a new foot position from position of legs when stretched out
                foot_position = leg_->zero_stance();

                //move the foot position to desired height of the robot
                foot_position.Translate(0.0f, 0.0f, -delta_height);

                //rotate the leg opposite the required orientation of the body
                foot_position.RotateX(-req_pose.orientation.roll);
                foot_position.RotateY(-req_pose.orientation.pitch);
                foot_position.RotateZ(-req_pose.orientation.yaw);
    
                leg_->transformToHip(foot_position);
            }
    };
}

#endif