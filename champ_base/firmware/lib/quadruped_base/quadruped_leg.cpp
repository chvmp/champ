#include <quadruped_leg.h>

QuadrupedLeg::QuadrupedLeg(Joint &hip_joint, Joint &upper_leg_joint, Joint &lower_leg_joint,  Joint &foot_joint):
    no_of_links_(0),
    center_to_nominal_(0),
    leg_id_(0),
    last_touchdown_(0),
    in_contact_(0),
    knee_direction_(0),
    is_pantograph_(false),
    gait_phase_(1),
    hip(&hip_joint),
    upper_leg(&upper_leg_joint),
    lower_leg(&lower_leg_joint),
    foot(&foot_joint)
{
    addLink(hip);
    addLink(upper_leg);
    addLink(lower_leg);
    addLink(foot);

    zero_stance_.X() = hip->x();
    zero_stance_.Y() = hip->y() + upper_leg->y() + lower_leg->y() + foot->y();
    zero_stance_.Z() = hip->z() + upper_leg->z() + lower_leg->z() + foot->z();

    center_to_nominal_ = sqrtf(pow(zero_stance_.X(),2) + pow(zero_stance_.Y(),2));
}

void QuadrupedLeg::addLink(Joint *l)
{
    joint_chain[no_of_links_++] = l;
}

Transformation QuadrupedLeg::foot_from_hip()
{
    //forward kinematics
    Transformation foot_position;
    foot_position = Identity<4,4>();

    for(unsigned int i = 3; i > 0; i--)
    {
        foot_position.Translate(joint_chain[i]->x(), joint_chain[i]->y(), joint_chain[i]->z());
        //prevent hip from being rotated as hip is on a different axis of rotation
        if(i > 1)
        {
            foot_position.RotateY(joint_chain[i-1]->theta());          
        }
    }

    return foot_position;
}

Transformation QuadrupedLeg::foot_from_base()
{
    Transformation foot_position;

    foot_position.p = foot_from_hip().p;
    foot_position.RotateX(hip->theta());

    foot_position.RotateX(hip->roll());
    foot_position.RotateY(hip->pitch());
    foot_position.RotateZ(hip->yaw());
    foot_position.Translate(hip->x(), hip->y(), hip->z());

    return foot_position;
}

void QuadrupedLeg::transformToHip(Transformation &foot)
{
    Point temp_point;

    temp_point.X() = foot.X() - hip->x();
    temp_point.Y() = foot.Y() - hip->y();
    temp_point.Z() = foot.Z();
    foot.p = temp_point;
}

void QuadrupedLeg::transformToBase(Transformation &foot)
{
    foot.RotateX(hip->roll());
    foot.RotateY(hip->pitch());
    foot.RotateZ(hip->yaw());
    foot.Translate(hip->x(), hip->y(), hip->z());
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
        joint_chain[i]->theta(joints[i]);
    }
}

Transformation QuadrupedLeg::zero_stance()
{
    return zero_stance_;
}

float QuadrupedLeg::center_to_nominal()
{
    return center_to_nominal_;
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

void QuadrupedLeg::gait_phase(bool phase)
{
    gait_phase_ =  phase;
}

bool QuadrupedLeg::gait_phase()
{
    return gait_phase_;
}

int QuadrupedLeg::knee_direction()
{
    return knee_direction_;
}

void QuadrupedLeg::knee_direction(int direction)
{
    knee_direction_ = direction;
}

void QuadrupedLeg::is_pantograph(bool config)
{
    is_pantograph_ = config;
}

bool QuadrupedLeg::is_pantograph()
{
    return is_pantograph_;
}