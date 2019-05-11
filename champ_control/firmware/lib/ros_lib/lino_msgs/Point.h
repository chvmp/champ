#ifndef _ROS_lino_msgs_Point_h
#define _ROS_lino_msgs_Point_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace lino_msgs
{

  class Point : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _point_type;
      _point_type point;

    Point():
      point()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->point.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->point.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "lino_msgs/Point"; };
    const char * getMD5(){ return "a7c84ff13976aa04656e56e300124444"; };

  };

}
#endif