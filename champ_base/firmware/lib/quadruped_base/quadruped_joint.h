#ifndef _QUADRUPED_JOINT_H_
#define _QUADRUPED_JOINT_H_

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
        Joint( float pos_x, float pos_y, float pos_z, float or_r, float or_p, float or_y);
        
        float theta();
        void theta(float angle);

        float x();
        float y();
        float z();

        float roll();
        float pitch();
        float yaw();
};

#endif