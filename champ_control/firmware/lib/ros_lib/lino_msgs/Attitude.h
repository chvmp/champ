#ifndef _ROS_lino_msgs_Attitude_h
#define _ROS_lino_msgs_Attitude_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace lino_msgs
{

  class Attitude : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _attitude_type;
      _attitude_type attitude;

    Attitude():
      attitude()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->attitude.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->attitude.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "lino_msgs/Attitude"; };
    const char * getMD5(){ return "ed3bd214a1cbf9efa188921b3d797bc3"; };

  };

}
#endif