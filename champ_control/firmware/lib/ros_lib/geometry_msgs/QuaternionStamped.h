#ifndef _ROS_geometry_msgs_QuaternionStamped_h
#define _ROS_geometry_msgs_QuaternionStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"

namespace geometry_msgs
{

  class QuaternionStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Quaternion _quaternion_type;
      _quaternion_type quaternion;

    QuaternionStamped():
      header(),
      quaternion()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->quaternion.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->quaternion.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/QuaternionStamped"; };
    const char * getMD5(){ return "e57f1e547e0e1fd13504588ffc8334e2"; };

  };

}
#endif