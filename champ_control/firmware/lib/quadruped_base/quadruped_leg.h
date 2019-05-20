#ifndef _QUADRUPPED_LEG_H_
#define  _QUADRUPPED_LEG_H_

#include <Geometry.h>
#include <revolute_joint.h>

class QuadrupedLeg: public RevoluteJoint
{
    Transformation ee_from_hip;
    Transformation ee_from_base;

    unsigned int no_of_links_;
    unsigned int leg_id_;

    void addLink(Link &l);

    public:
        float x; 
        float y; 
        float z; 
        float roll; 
        float pitch; 
        float yaw;

        Link *chain[3];

        QuadrupedLeg(RevoluteJoint &hip, RevoluteJoint &upper_leg, RevoluteJoint &lower_leg, 
                    float pos_x, float pos_y, float pos_z, 
                    float or_r, float or_p, float or_y);
        
        Transformation ee();
        Transformation ee_to_base();

        Transformation &forwardKinematics(Transformation &pose);

        void inverseKinematics(Transformation &target, float *joints);
        
        float &hip(){ return chain[0]->theta;};
        void hip(float joint_angle){chain[0]->theta = joint_angle;};

        float &upper_leg(){ return chain[1]->theta;};
        void upper_leg(float joint_angle){chain[1]->theta = joint_angle;};

        float &lower_leg(){ return chain[2]->theta;};
        void lower_leg(float joint_angle){chain[2]->theta = joint_angle;};

        void joints(float hip, float upper_leg, float lower_leg);
        void joints(float *joints);

        void ee_base_to_hip(Point &point);
};

#endif