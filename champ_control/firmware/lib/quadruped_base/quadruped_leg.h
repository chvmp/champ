#ifndef _QUADRUPPED_LEG_H_
#define _QUADRUPPED_LEG_H_

#include <Geometry.h>
#include <revolute_joint.h>

class QuadrupedLeg
{
    Transformation ee_from_hip_;
    Transformation ee_from_base_;

    unsigned int no_of_links_;
    unsigned int leg_id_;

    float x_; 
    float y_; 
    float z_; 

    float roll_; 
    float pitch_; 
    float yaw_;
    
    void addLink(RevoluteJoint *l);

    Transformation nominal_stance_;

    public:
        RevoluteJoint *hip;
        RevoluteJoint *upper_leg;
        RevoluteJoint *lower_leg;
        
        RevoluteJoint *chain[3];

        QuadrupedLeg(RevoluteJoint &hip_link, RevoluteJoint &upper_leg_link, RevoluteJoint &lower_leg_link, 
                    float pos_x, float pos_y, float pos_z, 
                    float or_r, float or_p, float or_y);
        
        Transformation forwardKinematics(Transformation &pose);

        Transformation ee();
        Transformation ee_to_base();
        
        void joints(float hip_joint, float upper_leg_joint, float lower_leg_joint);
        void joints(float *joints);

        float x();
        float y();
        float z();

        float roll();
        float pitch();
        float yaw();
        
        Transformation nominal_stance();
};

#endif