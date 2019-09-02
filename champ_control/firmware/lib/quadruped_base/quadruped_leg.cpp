#include<quadruped_leg.h>

QuadrupedLeg::QuadrupedLeg(Joint &hip_joint, Joint &upper_leg_joint, Joint &lower_leg_joint,  Joint &foot_joint):
    no_of_links_(0),
    leg_id_(0),
    last_touchdown_(0),
    in_contact_(0),
    hip(&hip_joint),
    upper_leg(&upper_leg_joint),
    lower_leg(&lower_leg_joint),
    foot(&foot_joint)
{
    addLink(hip);
    addLink(upper_leg);
    addLink(lower_leg);
    addLink(foot);

    nominal_stance_.X() = hip->x();
    nominal_stance_.Y() = hip->y() + upper_leg->z();
    nominal_stance_.Z() = -(lower_leg->x() + foot->x());
}

void QuadrupedLeg::addLink(Joint *l)
{
    chain[no_of_links_++] = l;
}

Transformation QuadrupedLeg::foot_from_hip()
{
    //forward kinematics
    Transformation foot_position;
    foot_position = Identity<4,4>();

    foot_position.Translate(chain[3]->x(), 0, 0);
    foot_position.RotateZ(chain[2]->theta());

    foot_position.Translate(chain[2]->x(), 0, 0);
    foot_position.RotateZ(chain[1]->theta());

    foot_position.Translate(0, 0, chain[1]->z());
    foot_position.RotateY(chain[0]->theta());

    return foot_position;
}

Transformation QuadrupedLeg::foot_from_base()
{
    Transformation foot_position;

    foot_position.p = foot_from_hip().p;
    
    foot_position.RotateX(hip->roll());
    foot_position.RotateY(hip->pitch());
    foot_position.RotateZ(hip->yaw());
    foot_position.Translate(hip->x(), hip->y(), hip->z());

    return foot_position;
}

void QuadrupedLeg::transformToHip(Transformation &foot)
{
    Point temp_point;
    temp_point.X() = -foot.Z();
    temp_point.Y() =  hip->x() - foot.X();
    temp_point.Z() = foot.Y() - hip->y();

    foot.p = temp_point;
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

void QuadrupedLeg::leg_id(unsigned int id)
{
    leg_id_ = id;
}

void QuadrupedLeg::in_contact(bool in_contact)
{
    if(!in_contact_ && in_contact){
        last_touchdown_ = micros();
    }
    in_contact_ = in_contact;
}

bool QuadrupedLeg::in_contact()
{
    return in_contact_;
}