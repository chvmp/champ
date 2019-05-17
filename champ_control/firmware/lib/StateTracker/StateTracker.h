class Joints
{

    public:
        float joints_[3];

        Joints()
        {
            for(int i = 0; i < 3; i++)
            {
                joints_[i] = 0;
            }
        }

        float &hip(){ return joints_[0];}
        void hip(float joint_angle){ joints_[0] = joint_angle;}

        float &upper_leg(){ return joints_[1];}
        void upper_leg(float joint_angle){ joints_[1] = joint_angle;}

        float &lower_leg(){ return joints_[2];}
        void lower_leg(float joint_angle){ joints_[2] = joint_angle;}

        void joints(float hip, float upper_leg, float lower_leg)
        { 
            joints_[0] = hip; 
            joints_[1] = upper_leg; 
            joints_[2] = lower_leg;
        }

        float *joints()
        {
            return joints_;
        }
};

class LegJoints
{   
    float *joints_[12];
    unsigned int total_joints_;
    
    void addLeg(Joints &joint)
    {
        joints_[total_joints_++] = &joint.hip();
        joints_[total_joints_++] = &joint.upper_leg();
        joints_[total_joints_++] = &joint.lower_leg();
    }

    public:
        Joints lf;
        Joints rf;
        Joints lh;
        Joints rh;

        LegJoints():
            total_joints_(0)
        {
            addLeg(lf);
            addLeg(rf);
            addLeg(lh);
            addLeg(rh);
        }

        float *joints()
        {
            return *joints_;
        }
};
