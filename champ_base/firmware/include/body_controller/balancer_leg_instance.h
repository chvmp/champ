#ifndef BALANCER_LEG_INSTANCE
#define BALANCER_LEG_INSTANCE

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_leg.h>

class BalancerLegInstance
{
    QuadrupedLeg *leg_;
    
    public:
        BalancerLegInstance(QuadrupedLeg *leg):
            leg_(leg)
        {
        }

        void balance(Transformation &foot_position, float body_roll, float body_pitch, 
                                float body_yaw, float target_z)
        {
            Transformation normal_vector;
            Transformation plane_p;
            Transformation normal_vector_origin;
            Transformation line_p0;
            Transformation line_p1;

            Point line_vector;
            float d;
            float delta_height = leg_->zero_stance().Z() + target_z;

            plane_p = leg_->zero_stance();
            plane_p.Translate(0, 0, -delta_height);

            line_p0 = leg_->zero_stance();
            line_p1 = leg_->zero_stance();
            line_p0.Translate(0, 0, -delta_height);
            line_p1.Translate(0, 0, leg_->lower_leg->x() + leg_->foot->x());

            normal_vector_origin = leg_->zero_stance();
            normal_vector_origin.Translate(-0.1, -0.1, -delta_height);
            normal_vector.p = normal_vector_origin.p;
            normal_vector.Translate(0, 0, 0.1);

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

        void poseCommand(Transformation &foot_position, float target_roll, float target_pitch, float target_yaw, float target_z)
        {
            float delta_height = leg_->zero_stance().Z() + target_z;

            foot_position = leg_->zero_stance();

            foot_position.Translate(0, 0, -delta_height);

            foot_position.RotateX(-target_roll);
            foot_position.RotateY(-target_pitch);
            foot_position.RotateZ(-target_yaw);

            leg_->transformToHip(foot_position);
        }
};

#endif