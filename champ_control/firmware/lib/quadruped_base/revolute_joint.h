#ifndef _REVOLUTE_JOINT_H_
#define _REVOLUTE_JOINT_H_

class RevoluteJoint
{
    float d_;
    float theta_;
    float r_;
    float alpha_;

    public:
        RevoluteJoint(float d, float theta, float r, float alpha);
        
        float &theta();
        void theta(float angle);
        float &d();
        float &r();
        float &alpha();
};

#endif