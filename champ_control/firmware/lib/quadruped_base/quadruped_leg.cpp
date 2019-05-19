#include<quadruped_leg.h>

QuadrupedLeg::QuadrupedLeg(RevoluteJoint &hip, RevoluteJoint &upper_leg, RevoluteJoint &lower_leg):
    no_of_links_(0),
    RevoluteJoint(d, theta, r, alpha)
{
    addLink(hip);
    addLink(upper_leg);
    addLink(lower_leg);
}

void QuadrupedLeg::QuadrupedLeg::addLink(Link &l)
{
    chain_[no_of_links_++] = &l;
}

Transformation &QuadrupedLeg::forwardKinematics(Transformation &pose)
{
    for(int i = no_of_links_ - 1; i >= 0; i--)
    {
        pose.RotateX(chain_[i]->alpha);
        pose.Translate(chain_[i]->r, 0,0);
        pose.Translate(0,0,chain_[i]->d);
        pose.RotateZ(chain_[i]->theta);
    }
    pose.RotateX(-1.5708);

    return pose;
}

Transformation QuadrupedLeg::ee()
{
    currentPose = Identity<4,4>();
    return forwardKinematics(currentPose);
}

void QuadrupedLeg::inverseKinematics(Transformation &target, float *joints)
{
    joints[0] = -(atan(target.p.Z() / target.p.X()) - ( 1.5708 - acos(chain_[1]->d / sqrt(pow(target.p.Z(),2) + pow(target.p.X(), 2)))));
    target.RotateY(joints[0]);
    // // ik for knee forward
    // // joints[2] = acos( (pow(target.p.X(),2) + pow(target.Y(),2) - pow(chain_[1]->r ,2) - pow(chain_[2]->r ,2)) / (2 * chain_[1]->r * chain_[2]->r) );
    // // joints[1] = atan(target.p.Y() / target.p.X()) - atan( (chain_[2]->r * sin(joints[2])) / (chain_[1]->r + (chain_[2]->r * cos(joints[2]))));

    // // reverse
    joints[2] = -acos((pow(target.p.X(),2) + pow(target.Y(),2) - pow(chain_[1]->r ,2) - pow(chain_[2]->r ,2)) / (2 * chain_[1]->r * chain_[2]->r));
    joints[1] = (atan(target.p.Y() / target.p.X()) - atan( (chain_[2]->r * sin(joints[2])) / (chain_[1]->r + (chain_[2]->r * cos(joints[2])))));
    target.RotateY(-joints[0]);
} 

void QuadrupedLeg::joints(float hip, float upper_leg, float lower_leg)
{ 
    chain_[0]->theta = hip; 
    chain_[1]->theta = upper_leg; 
    chain_[2]->theta = lower_leg;
};

void QuadrupedLeg::joints(float *joints)
{
    for(int i = 0; i < 3; i++)
    {
        chain_[i]->theta = joints[i];
    }
};