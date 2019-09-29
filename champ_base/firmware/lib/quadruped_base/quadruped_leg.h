#ifndef _QUADRUPPED_LEG_H_
#define _QUADRUPPED_LEG_H_

#include <Geometry.h>
#include <quadruped_joint.h>
#include <Arduino.h>

class QuadrupedLeg
{
    unsigned int no_of_links_;
    
    void addLink(Joint *l);

    Transformation nominal_stance_;

    unsigned int leg_id_;

    unsigned long last_touchdown_;

    bool in_contact_;

    int knee_direction_;

    public:
        QuadrupedLeg(Joint &hip_joint, Joint &upper_leg_joint, Joint &lower_leg_link, Joint &foot_joint);
        
        Transformation foot_from_hip();
        Transformation foot_from_base();
        
        void joints(float hip_joint, float upper_leg_joint, float lower_leg_joint);
        void joints(float *joints);

        Transformation nominal_stance();
        unsigned int leg_id();

        unsigned long int last_touchdown();
        void last_touchdown(unsigned long int current_time);

        void transformToHip(Transformation &foot);
        void transformToBase(Transformation &foot);

        void leg_id(unsigned int id);
        void in_contact(bool in_contact);
        bool in_contact();

        int knee_direction();
        void knee_direction(int direction);
        
        Joint *hip;
        Joint *upper_leg;
        Joint *lower_leg;
        Joint *foot;

        Joint *joint_chain[4];
};

#endif

