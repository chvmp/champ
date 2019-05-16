#include <Geometry.h>

class Link
{
    public:
        float d, theta, r, alpha;

        Link(float _d, float _theta, float _r, float _alpha) : d(_d), theta(_theta), r(_r), alpha(_alpha) { }
        virtual void Move(float amount) = 0;
};

class RevoluteJoint : public Link
{
    public:
        RevoluteJoint(float d, float theta, float r, float alpha) : Link(d, theta, r, alpha) { }
        void Move(float amount) { theta += amount;}
};    

class QuadrupedLeg
{
    Transformation currentPose;
    unsigned int no_of_links;
    public:

        Link *chain[3];
        QuadrupedLeg(RevoluteJoint &hip, RevoluteJoint &upper_leg, RevoluteJoint &lower_leg);
        void addLink(Link &l);//float d, float theta, float r, float alpha, void (*move)(Link&, float))
        Transformation &forwardKinematics(Transformation &pose);
        Transformation forwardKinematics();
        void inverseKinematics(Transformation &target, float *joints);
        void updateJoints(float *joints);
};





