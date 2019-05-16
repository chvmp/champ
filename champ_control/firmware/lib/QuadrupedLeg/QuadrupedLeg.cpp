#include<QuadrupedLeg.h>

QuadrupedLeg::QuadrupedLeg(RevoluteJoint &hip, RevoluteJoint &upper_leg, RevoluteJoint &lower_leg):
    no_of_links(0)
{
    addLink(hip);
    addLink(upper_leg);
    addLink(lower_leg);
}

void QuadrupedLeg::QuadrupedLeg::addLink(Link &l)
{
    chain[no_of_links++] = &l;
}

// // Transforms pose from the end effector coordinate frame to the base coordinate frame.
Transformation &QuadrupedLeg::forwardKinematics(Transformation &pose)
{
    for(int i = no_of_links - 1; i >= 0; i--)
    {
        pose.RotateX(chain[i]->alpha);
        pose.Translate(chain[i]->r, 0,0);
        pose.Translate(0,0,chain[i]->d);
        pose.RotateZ(chain[i]->theta);
    }
    pose.RotateX(-1.5708);

    return pose;
}

Transformation QuadrupedLeg::forwardKinematics()
{
    currentPose = Identity<4,4>();
    return forwardKinematics(currentPose);
}

void QuadrupedLeg::inverseKinematics(Transformation &target, float *joints){
    joints[0] = -(atan(target.p.Z() / target.p.X()) - ( 1.5708 - acos(chain[1]->d / sqrt(pow(target.p.Z(),2) + pow(target.p.X(), 2)))));
    target.RotateY(joints[0]);
    // // ik for knee forward
    // // joints[2] = acos( (pow(target.p.X(),2) + pow(target.Y(),2) - pow(chain[1]->r ,2) - pow(chain[2]->r ,2)) / (2 * chain[1]->r * chain[2]->r) );
    // // joints[1] = atan(target.p.Y() / target.p.X()) - atan( (chain[2]->r * sin(joints[2])) / (chain[1]->r + (chain[2]->r * cos(joints[2]))));

    // // reverse
    joints[2] = -acos((pow(target.p.X(),2) + pow(target.Y(),2) - pow(chain[1]->r ,2) - pow(chain[2]->r ,2)) / (2 * chain[1]->r * chain[2]->r));
    joints[1] = (atan(target.p.Y() / target.p.X()) - atan( (chain[2]->r * sin(joints[2])) / (chain[1]->r + (chain[2]->r * cos(joints[2])))));
    target.RotateY(-joints[0]);
} 

void QuadrupedLeg::updateJoints(float *joints)
{
    for(int i = 0 - 1; i <= no_of_links; i++)
    {
        chain[i]->theta = joints[i];
    }
}