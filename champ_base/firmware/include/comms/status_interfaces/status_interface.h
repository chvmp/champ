#ifndef STATUS_H
#define STATUS_H


#include <champ_msgs/Velocities.h>
#include <champ_msgs/Pose.h>
#include <champ_msgs/Joints.h>
#include <geometry/geometry.h>

namespace champ
{
    namespace Interfaces
    {
        template<class StatusInterface>

        class Status
        {
                StatusInterface *status_interface_;
                public:
                    
                    Status(StatusInterface &interface):
                        status_interface_(&interface)
                    {
                        
                    }

                    void publishPoints(Transformation foot_positions[4])
                    {
                        status_interface_->publishPoints(foot_positions);
                    }

                    void publishVelocities(champ::Velocities vel)
                    {
                        status_interface_->publishVelocities(vel);
                    }

                    void publishJointStates(float joint_positions[12])
                    {
                        status_interface_->publishJointStates(joint_positions);
                    }

                    void publishIMU(champ::Orientation &rotation, champ::Accelerometer &accel, 
                                    champ::Gyroscope &gyro, champ::Magnetometer &mag)
                    {
                        status_interface_->publishIMU(rotation, accel, gyro, mag);
                    }
        };
    }
}
#endif