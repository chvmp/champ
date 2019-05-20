#include<quadruped_balancer.h>

QuadrupedBalancer::QuadrupedBalancer()
{
}

void QuadrupedBalancer::balance(QuadrupedBase &base, float target_roll, float target_pitch, 
                        float target_yaw, float target_x, float target_y, float target_z)
{
    lf_stance_.p = base.lf->nominal_stance();
    lf_stance_.RotateX(-target_roll);
    lf_stance_.RotateY(-target_pitch);
    lf_stance_.RotateZ(-target_yaw);
    lf_stance_.Translate(-target_x, -target_y, - target_z);
    ee_base_to_hip(base.lf, lf_stance_.p);

    rf_stance_.p = base.rf->nominal_stance();
    rf_stance_.RotateX(-target_roll);
    rf_stance_.RotateY(-target_pitch);
    rf_stance_.RotateZ(-target_yaw);
    rf_stance_.Translate(-target_x, -target_y, - target_z);
    ee_base_to_hip(base.rf, rf_stance_.p);

    lh_stance_.p = base.lh->nominal_stance();
    lh_stance_.RotateX(-target_roll);
    lh_stance_.RotateY(-target_pitch);
    lh_stance_.RotateZ(-target_yaw);
    lh_stance_.Translate(-target_x, -target_y, - target_z);
    ee_base_to_hip(base.lh, lh_stance_.p);

    rh_stance_.p = base.rh->nominal_stance();
    rh_stance_.RotateX(-target_roll);
    rh_stance_.RotateY(-target_pitch);
    rh_stance_.RotateZ(-target_yaw);
    rh_stance_.Translate(-target_x, -target_y, - target_z);
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