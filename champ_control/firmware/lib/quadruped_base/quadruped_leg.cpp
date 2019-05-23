#include<quadruped_leg.h>

QuadrupedLeg::QuadrupedLeg(RevoluteJoint &hip_link, RevoluteJoint &upper_leg_link, RevoluteJoint &lower_leg_link, 
    float pos_x, float pos_y, float pos_z, float or_r, float or_p, float or_y):
    no_of_links_(0),
    leg_id_(0),
    x_(pos_x), 
    y_(pos_y),
    z_(pos_z),
    roll_(or_r),
    pitch_(or_p),
    yaw_(or_y),
    hip(&hip_link),
    upper_leg(&upper_leg_link),
    lower_leg(&lower_leg_link)
{
    addLink(hip);
    addLink(upper_leg);
    addLink(lower_leg);
    nominal_stance_ = ee_to_base();
}

void QuadrupedLeg::addLink(RevoluteJoint *l)
{
    chain[no_of_links_++] = l;
}

Transformation QuadrupedLeg::forwardKinematics(Transformation &pose)
{
    for(int i = no_of_links_ - 1; i >= 0; i--)
    {
        pose.RotateX(chain[i]->alpha());
        pose.Translate(chain[i]->r(), 0, 0);
        pose.Translate(0, 0, chain[i]->d());
        pose.RotateZ(chain[i]->theta());
    }
    pose.RotateX(-PI/2);

    return pose;
}

Transformation QuadrupedLeg::ee()
{
    ee_from_hip_ = Identity<4,4>();

    return forwardKinematics(ee_from_hip_);
}

Transformation QuadrupedLeg::ee_to_base()
{
    ee_from_base_.p = ee().p;
    ee_from_base_.RotateZ(roll_);
    ee_from_base_.RotateY(pitch_);
    ee_from_base_.RotateZ(yaw_);
    ee_from_base_.Translate(x_, y_, z_);

    return ee_from_base_;
}

void QuadrupedLeg::ee_base_to_hip(Point &point)
{
    Point temp_point;
    temp_point.X() = -point.Z();
    temp_point.Y() = x_ - point.X();
    temp_point.Z() = point.Y() - y_;

    point = temp_point;
}

void QuadrupedLeg::joints(float hip_joint, float upper_leg_joint, float lower_leg_joint)
{ 
    hip->theta(hip_joint);
    upper_leg->theta(upper_leg_joint);
    lower_leg->theta(lower_leg_joint);
}

void QuadrupedLeg::joints(float *joints)
{
    for(unsigned int i = 0; i < 3; i++)
    {
        chain[i]->theta(joints[i]);
    }
}

float QuadrupedLeg::x()
{
    return x_;
}

float QuadrupedLeg::y()
{
    return y_;
}

float QuadrupedLeg::z()
{
    return z_;
}

float QuadrupedLeg::roll()
{
    return roll_;
}

float QuadrupedLeg::pitch()
{
    return pitch_;
}

float QuadrupedLeg::yaw()
{
    return yaw_;
}

Transformation QuadrupedLeg::nominal_stance()
{
    return nominal_stance_;
}