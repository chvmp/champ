#include<quadruped_leg.h>

QuadrupedLeg::QuadrupedLeg(RevoluteJoint &hip_link, RevoluteJoint &upper_leg_link, RevoluteJoint &lower_leg_link, 
    float pos_x, float pos_y, float pos_z, float or_r, float or_p, float or_y):
    no_of_links_(0),
    x_(pos_x), 
    y_(pos_y),
    z_(pos_z),
    roll_(or_r),
    pitch_(or_p),
    yaw_(or_y),
    leg_id_(0),
    hip(&hip_link),
    upper_leg(&upper_leg_link),
    lower_leg(&lower_leg_link)
{
    addLink(hip);
    addLink(upper_leg);
    addLink(lower_leg);
    nominal_stance_ = foot_from_base();
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

Transformation QuadrupedLeg::foot()
{
    foot_from_hip_ = Identity<4,4>();

    return forwardKinematics(foot_from_hip_);
}

Transformation QuadrupedLeg::foot_from_base()
{
    foot_from_base_.p = foot().p;
    foot_from_base_.RotateX(roll_);
    foot_from_base_.RotateY(pitch_);
    foot_from_base_.RotateZ(yaw_);
    foot_from_base_.Translate(x_, y_, z_);

    return foot_from_base_;
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

unsigned int  QuadrupedLeg::leg_id()
{
    return leg_id_;
}

unsigned long int  QuadrupedLeg::last_touchdown()
{
    return last_touchdown_;
}

void  QuadrupedLeg::last_touchdown(unsigned long int current_time)
{
    last_touchdown_ = current_time;
}

void QuadrupedLeg::transformToHip(Transformation &foot)
{
    Point temp_point;
    temp_point.X() = - foot.Z();
    temp_point.Y() = x_ - foot.X();
    temp_point.Z() = foot.Y() - y_;

    foot.p = temp_point;
}

void QuadrupedLeg::setLegID(unsigned int id)
{
    leg_id_ = id;
}

void QuadrupedLeg::updateGroundContact(bool in_contact)
{
    in_contact_ = in_contact;
    if(!in_contact_ && in_contact){
        last_touchdown_ = micros();
    }
}