#include<revolute_joint.h>

RevoluteJoint::RevoluteJoint(float d, float theta, float r, float alpha):
    d_(d),
    theta_(theta),
    r_(r),
    alpha_(alpha)
{ 

} 

float &RevoluteJoint::theta()
{ 
    return theta_; 
}

void RevoluteJoint::theta(float angle)
{ 
    theta_ = angle; 
}

float &RevoluteJoint::d()
{ 
    return d_; 
}

float &RevoluteJoint::r()
{ 
    return r_; 
}

float &RevoluteJoint::alpha()
{ 
    return alpha_; 
}