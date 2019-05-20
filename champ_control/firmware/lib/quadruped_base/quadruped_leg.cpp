#include<quadruped_leg.h>

QuadrupedLeg::QuadrupedLeg(RevoluteJoint &hip, RevoluteJoint &upper_leg, RevoluteJoint &lower_leg, 
    float pos_x, float pos_y, float pos_z, float or_r, float or_p, float or_y):
    no_of_links_(0),
    RevoluteJoint(d, theta, r, alpha),
    x(pos_x), 
    y(pos_y),
    z(pos_z),
    roll(or_r),
    pitch(or_p),
    yaw(or_y)
{
    addLink(hip);
    addLink(upper_leg);
    addLink(lower_leg);
}

void QuadrupedLeg::QuadrupedLeg::addLink(Link &l)
{
    chain[no_of_links_++] = &l;
}

Transformation &QuadrupedLeg::forwardKinematics(Transformation &pose)
{
    for(int i = no_of_links_ - 1; i >= 0; i--)
    {
        pose.RotateX(chain[i]->alpha);
        pose.Translate(chain[i]->r, 0,0);
        pose.Translate(0,0,chain[i]->d);
        pose.RotateZ(chain[i]->theta);
    }
    pose.RotateX(-PI/2);

    return pose;
}

Transformation QuadrupedLeg::ee()
{
    ee_from_hip = Identity<4,4>();

    return forwardKinematics(ee_from_hip);
}

Transformation QuadrupedLeg::ee_to_base()
{
    ee_from_base.p = ee().p;
    ee_from_base.RotateY(-pitch);
    ee_from_base.RotateZ(yaw);
    ee_from_base.Translate(x, y, z);

    return ee_from_base;
}

void QuadrupedLeg::ee_base_to_hip(Point &point)
{
    Point temp_point;
    temp_point.X() = -point.Z();
    temp_point.Y() = x - point.X();
    temp_point.Z() = point.Y() - y;

    point = temp_point;
}

void QuadrupedLeg::joints(float hip, float upper_leg, float lower_leg)
{ 
    chain[0]->theta = hip; 
    chain[1]->theta = upper_leg; 
    chain[2]->theta = lower_leg;
};

void QuadrupedLeg::joints(float *joints)
{
    for(int i = 0; i < 3; i++)
    {
        chain[i]->theta = joints[i];
    }
};