#ifndef _ROS_geometry_msgs_Accel_h
#define _ROS_geometry_msgs_Accel_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace geometry_msgs
{

  class Accel : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _linear_type;
      _linear_type linear;
      typedef geometry_msgs::Vector3 _angular_type;
      _angular_type angular;

    Accel():
      linear(),
      angular()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->linear.serialize(outbuffer + offset);
      offset += this->angular.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->linear.deserialize(inbuffer + offset);
      offset += this->angular.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Accel"; };
    const char * getMD5(){ return "9f195f881246fdfa2798d1d3eebca84a"; };

  };

}
#endif