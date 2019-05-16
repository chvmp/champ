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

template<int max_links> class QuadrupedLeg
{
    Transformation currentPose;
    unsigned int no_of_links;
    // unsigned int max_links;

    public:

        Link *chain[max_links];
        // QuadrupedLeg();
        // void addLink(Link &l);//float d, float theta, float r, float alpha, void (*move)(Link&, float))
        // Transformation &getFootPose(Transformation &pose);
        // Transformation &getFootPose();
        // void getJointAngles(Transformation &target, double *joints);

        QuadrupedLeg()
        {
            no_of_links = 0;
            // addLink(hip);
            // addLink(upper_leg);
            // addLink(lower_leg);
        }

        // // Add a link - it's D-H parameters as well as a function pointer describing it's joint movement
        void addLink(Link &l)
        {
            if(no_of_links == max_links)
                return;

            chain[no_of_links++] = &l;
        }

        // // Transforms pose from the end effector coordinate frame to the base coordinate frame.
        Transformation &getFootPose(Transformation &pose)
        {
            for(int i = no_of_links - 1; i >= 0; i--)
            {
                // These four operations will convert between two coordinate frames defined by D-H parameters, it's pretty standard stuff
                pose.RotateX(chain[i]->alpha);
                pose.Translate(chain[i]->r, 0,0);
                pose.Translate(0,0,chain[i]->d);
                pose.RotateZ(chain[i]->theta);
            }
            pose.RotateX(-1.5708);

            return pose;
        }

        // // Handy overload to save having to feed in a fresh Transformation every time
        Transformation getFootPose()
        {
            currentPose = Identity<4,4>();
            return getFootPose(currentPose);
        }

        void getJointAngles(Transformation &target, double *joints){
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
};





