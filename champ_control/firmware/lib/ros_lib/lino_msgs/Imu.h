#ifndef _ROS_lino_msgs_Imu_h
#define _ROS_lino_msgs_Imu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace lino_msgs
{

  class Imu : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _linear_acceleration_type;
      _linear_acceleration_type linear_acceleration;
      typedef geometry_msgs::Vector3 _angular_velocity_type;
      _angular_velocity_type angular_velocity;
      typedef geometry_msgs::Vector3 _magnetic_field_type;
      _magnetic_field_type magnetic_field;

    Imu():
      linear_acceleration(),
      angular_velocity(),
      magnetic_field()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      offset += this->angular_velocity.serialize(outbuffer + offset);
      offset += this->magnetic_field.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      offset += this->magnetic_field.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "lino_msgs/Imu"; };
    const char * getMD5(){ return "275110405f08e1b7c0c0f1aba3e19c67"; };

  };

}
#endif