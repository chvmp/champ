#include<quadruped_joint.h>

Joint::Joint( float pos_x, float pos_y, float pos_z, float or_r, float or_p, float or_y):
    x_(pos_x), 
    y_(pos_y),
    z_(pos_z),
    roll_(or_r),
    pitch_(or_p),
    yaw_(or_y),
    theta_(0)
{ 
} 

float Joint::theta()
{ 
    return theta_; 
}

void Joint::theta(float angle)
{ 
    theta_ = angle; 
}

float Joint::x()
{
    return x_;
}

float Joint::y()
{
    return y_;
}

float Joint::z()
{
    return z_;
}

float Joint::roll()
{
    return roll_;
}

float Joint::pitch()
{
    return pitch_;
}

float Joint::yaw()
{
    return yaw_;
}