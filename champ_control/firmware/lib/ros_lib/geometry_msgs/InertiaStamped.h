#ifndef _ROS_geometry_msgs_InertiaStamped_h
#define _ROS_geometry_msgs_InertiaStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Inertia.h"

namespace geometry_msgs
{

  class InertiaStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Inertia _inertia_type;
      _inertia_type inertia;

    InertiaStamped():
      header(),
      inertia()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->inertia.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->inertia.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/InertiaStamped"; };
    const char * getMD5(){ return "ddee48caeab5a966c5e8d166654a9ac7"; };

  };

}
#endif