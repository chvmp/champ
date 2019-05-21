#include<quadruped_balancer.h>
#include <BasicLinearAlgebra.h>

QuadrupedBalancer::QuadrupedBalancer()
{
}

void QuadrupedBalancer::balance(QuadrupedBase &base, float target_roll, float target_pitch, 
                        float target_yaw, float target_x, float target_y, float target_z)
{
    Transformation normal_vector;
    Transformation p0;
    Transformation p1;
    float d;
    p0.p = base.lf->nominal_stance();
    p1.p = base.lf->nominal_stance();
    p1.Translate(0, 0,-target_z);
    normal_vector.Z() = p1.Z() + 0.05;

    p0.RotateX(-target_roll);
    p1.RotateX(-target_roll);
    normal_vector.RotateX(-target_roll);

    p0.RotateY(-target_pitch);
    p1.RotateY(-target_pitch);
    normal_vector.RotateY(-target_pitch);
    
    p0.RotateZ(-target_yaw);
    p1.RotateZ(-target_yaw);
    normal_vector.RotateZ(-target_yaw);

    d = p1.X() * normal_vector.p.X() + \
        p1.Y() * normal_vector.p.Y() + \
        p1.Z() * normal_vector.p.Z();

    // ee_base_to_hip(base.lf, normal_vector.p);
    // lf_stance_.p = normal_vector.p;
    BLA::Matrix<4,4> denominator = 
    {
    normal_vector.X(), normal_vector.Y(),   normal_vector.Z(),                  0,
                    1,                 0,                   0, -(p1.X() - p0.X()),
                    0,                 1,                   0, -(p1.Y() - p0.Y()),
                    0,                 0,                   1, -(p1.Z() - p0.Z())
    };

    BLA::Matrix<4,4> x_numerator = 
    {
         d, normal_vector.Y(),   normal_vector.Z(),                  0,
    p0.X(),                 0,                   0, -(p1.X() - p0.X()),
    p0.Y(),                 1,                   0, -(p1.Y() - p0.Y()),
    p0.Z(),                 0,                   1, -(p1.Z() - p0.Z())
    };

    BLA::Matrix<4,4> y_numerator = 
    {
    normal_vector.X(),      d,   normal_vector.Z(),                  0,
                    1, p0.X(),                   0, -(p1.X() - p0.X()),
                    0, p0.Y(),                   0, -(p1.Y() - p0.Y()),
                    0, p0.Z(),                   1, -(p1.Z() - p0.Z())
    };

    BLA::Matrix<4,4> z_numerator = 
    {
    normal_vector.X(), normal_vector.Y(),      d,                  0,
                    1,                 0, p0.X(), -(p1.X() - p0.X()),
                    0,                 1, p0.Y(), -(p1.Y() - p0.Y()),
                    0,                 0, p0.Z(), -(p1.Z() - p0.Z())
    };

    BLA::Matrix<4,4> t_numerator = 
    {
    normal_vector.X(), normal_vector.Y(),   normal_vector.Z(),      d,
                    1,                 0,                   0, p0.X(),
                    0,                 1,                   0, p0.Y(),
                    0,                 0,                   1, p0.Z()
    };

    lf_stance_.p.X() = Determinant(x_numerator) / Determinant(denominator);
    lf_stance_.p.Y() = Determinant(y_numerator) / Determinant(denominator);
    lf_stance_.p.Z() = Determinant(z_numerator) / Determinant(denominator);
    ee_base_to_hip(base.lf, lf_stance_.p);

// d,p0.X(), p0.Y(), p0.Z();

    // Point test;
    // Rotation r_test;
    // test(0) = 0;
    // test(1) = 0;
    // test(2) = 1;
    //     r_test.RotateY(1.5708);
    // test = r_test * test;
    // lf_stance_.Translate(0, 0, 1);

    // lf_stance_.RotateX(-target_roll);
    // lf_stance_.RotateY(1.5708);
    // lf_stance_.RotateZ(-target_yaw);
    // lf_stance_.Translate(-target_x, -target_y, - target_z);
    // ee_base_to_hip(base.lf, lf_stance_.p);
    // lf_stance_.p = test;

    rf_stance_.p = base.rf->nominal_stance();
    // rf_stance_.RotateX(-target_roll);
    // rf_stance_.RotateY(-target_pitch);
    // rf_stance_.RotateZ(-target_yaw);
    // rf_stance_.Translate(-target_x, -target_y, - target_z);
    ee_base_to_hip(base.rf, rf_stance_.p);

    lh_stance_.p = base.lh->nominal_stance();
    // lh_stance_.RotateX(-target_roll);
    // lh_stance_.RotateY(-target_pitch);
    // lh_stance_.RotateZ(-target_yaw);
    // lh_stance_.Translate(-target_x, -target_y, - target_z);
    ee_base_to_hip(base.lh, lh_stance_.p);

    rh_stance_.p = base.rh->nominal_stance();
    // rh_stance_.RotateX(-target_roll);
    // rh_stance_.RotateY(-target_pitch);
    // rh_stance_.RotateZ(-target_yaw);
    // rh_stance_.Translate(-target_x, -target_y, - target_z);
    ee_base_to_hip(base.rh, rh_stance_.p);
}

void QuadrupedBalancer::ee_base_to_hip(QuadrupedLeg *leg, Point &point)
{
    Point temp_point;
    temp_point.X() = -point.Z();
    temp_point.Y() = leg->x() - point.X();
    temp_point.Z() = point.Y() - leg->y();

    point = temp_point;
}

Transformation QuadrupedBalancer::lf_stance()
{   
    return lf_stance_;
}

Transformation QuadrupedBalancer::rf_stance()
{
    return rf_stance_;
}

Transformation QuadrupedBalancer::lh_stance()
{
    return lh_stance_;
}

Transformation QuadrupedBalancer::rh_stance()
{
    return rh_stance_;
}