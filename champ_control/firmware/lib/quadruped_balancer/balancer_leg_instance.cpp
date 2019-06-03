#include<balancer_leg_instance.h>
#include<BasicLinearAlgebra.h>

BalancerLegInstance::BalancerLegInstance(QuadrupedLeg *leg):
    leg_(leg)
{
}

void BalancerLegInstance::legGroundIntersection(float target_roll, float target_pitch, 
                        float target_yaw, float target_x, float target_y, float target_z)
{
    Transformation normal_vector;
    Transformation plane_p;
    Transformation normal_vector_origin;
    Transformation line_p1;
    Point line_vector;
    float d;
    
    plane_p = leg_->nominal_stance();
    plane_p.Translate(0, 0, -target_z);

    leg_->nominal_stance() = leg_->nominal_stance();
    line_p1 = leg_->nominal_stance();
    line_p1.Translate(0, 0, 0.05);
    
    normal_vector_origin = leg_->nominal_stance();
    normal_vector_origin.Translate(-0.1, -0.1, -target_z);
    normal_vector.p = normal_vector_origin.p;
    normal_vector.Translate(0, 0, 0.1);

    plane_p.RotateX(target_roll);
    plane_p.RotateY(target_pitch);
    plane_p.RotateZ(target_yaw);

    normal_vector_origin.RotateX(target_roll);
    normal_vector_origin.RotateY(target_pitch);
    normal_vector_origin.RotateZ(target_yaw);

    normal_vector.RotateX(target_roll);
    normal_vector.RotateY(target_pitch);
    normal_vector.RotateZ(target_yaw);

    normal_vector.X() = normal_vector.X() - normal_vector_origin.X();
    normal_vector.Y() = normal_vector.Y() - normal_vector_origin.Y();
    normal_vector.Z() = normal_vector.Z() - normal_vector_origin.Z();

    line_vector.X() = -(line_p1.X() - leg_->nominal_stance().X());
    line_vector.Y() = -(line_p1.Y() - leg_->nominal_stance().Y());
    line_vector.Z() = -(line_p1.Z() - leg_->nominal_stance().Z());

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
        leg_->nominal_stance().X(),                 0,                   0, line_vector.X(),
        leg_->nominal_stance().Y(),                 1,                   0, line_vector.Y(),
        leg_->nominal_stance().Z(),                 0,                   1, line_vector.Z()
    };

    BLA::Matrix<4,4> y_numerator = 
    {
        normal_vector.X(),                         d,   normal_vector.Z(),               0,
                        1, leg_->nominal_stance().X(),                   0, line_vector.X(),
                        0, leg_->nominal_stance().Y(),                   0, line_vector.Y(),
                        0, leg_->nominal_stance().Z(),                   1, line_vector.Z()
    };

    BLA::Matrix<4,4> z_numerator = 
    {
        normal_vector.X(), normal_vector.Y(),                         d,               0,
                        1,                 0, leg_->nominal_stance().X(), line_vector.X(),
                        0,                 1, leg_->nominal_stance().Y(), line_vector.Y(),
                        0,                 0, leg_->nominal_stance().Z(), line_vector.Z()
    };

    BLA::Matrix<4,4> t_numerator = 
    {
        normal_vector.X(), normal_vector.Y(),   normal_vector.Z(),                         d,
                        1,                 0,                   0, leg_->nominal_stance().X(),
                        0,                 1,                   0, leg_->nominal_stance().Y(),
                        0,                 0,                   1, leg_->nominal_stance().Z()
    };

    foot_.p.X() = x_numerator.Det() / denominator.Det();
    foot_.p.Y() = y_numerator.Det() / denominator.Det();
    foot_.p.Z() = z_numerator.Det() / denominator.Det();
    ee_base_to_hip(foot_);
}

void BalancerLegInstance::ee_base_to_hip(Transformation &foot)
{
    Point temp_point;
    temp_point.X() = - foot.Z();
    temp_point.Y() = leg_->x() - foot.X();
    temp_point.Z() = foot.Y() - leg_->y();

    foot.p = temp_point;
}

Transformation BalancerLegInstance::stance()
{   
    return foot_;
}
