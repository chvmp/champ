#ifndef QUADRUPED_JOINT_H
#define QUADRUPED_JOINT_H

class Joint
{
    float x_; 
    float y_; 
    float z_; 

    float roll_; 
    float pitch_; 
    float yaw_;

    float theta_;

    public:
        Joint( float pos_x, float pos_y, float pos_z, float or_r, float or_p, float or_y):
            x_(pos_x), 
            y_(pos_y),
            z_(pos_z),
            roll_(or_r),
            pitch_(or_p),
            yaw_(or_y),
            theta_(0)
        { 
        } 

        float theta()
        { 
            return theta_; 
        }

        void theta(float angle)
        { 
            theta_ = angle; 
        }

        float x()
        {
            return x_;
        }

        float y()
        {
            return y_;
        }

        float z()
        {
            return z_;
        }

        float roll()
        {
            return roll_;
        }

        float pitch()
        {
            return pitch_;
        }

        float yaw()
        {
            return yaw_;
        }
};

#endif