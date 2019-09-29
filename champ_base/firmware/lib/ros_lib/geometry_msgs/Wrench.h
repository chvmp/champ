#ifndef _ROS_geometry_msgs_Wrench_h
#define _ROS_geometry_msgs_Wrench_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace geometry_msgs
{

  class Wrench : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _force_type;
      _force_type force;
      typedef geometry_msgs::Vector3 _torque_type;
      _torque_type torque;

    Wrench():
      force(),
      torque()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->force.serialize(outbuffer + offset);
      offset += this->torque.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->force.deserialize(inbuffer + offset);
      offset += this->torque.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Wrench"; };
    const char * getMD5(){ return "4f539cf138b23283b520fd271b567936"; };

  };

}
#endif