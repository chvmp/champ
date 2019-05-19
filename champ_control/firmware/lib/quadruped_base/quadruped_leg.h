#include <Geometry.h>
#include <revolute_joint.h>

class QuadrupedLeg: public RevoluteJoint
{
    Transformation currentPose;

    unsigned int no_of_links_;
    unsigned int leg_id_;

    Link *chain_[3];

    void addLink(Link &l);

    public:
        QuadrupedLeg(RevoluteJoint &hip, RevoluteJoint &upper_leg, RevoluteJoint &lower_leg);
        
        Transformation ee();
        Transformation &forwardKinematics(Transformation &pose);

        void inverseKinematics(Transformation &target, float *joints);
        
        float &hip(){ return chain_[0]->theta;};
        void hip(float joint_angle){chain_[0]->theta = joint_angle;};

        float &upper_leg(){ return chain_[1]->theta;};
        void upper_leg(float joint_angle){chain_[1]->theta = joint_angle;};

        float &lower_leg(){ return chain_[2]->theta;};
        void lower_leg(float joint_angle){chain_[2]->theta = joint_angle;};

        void joints(float hip, float upper_leg, float lower_leg);
        void joints(float *joints);
};

