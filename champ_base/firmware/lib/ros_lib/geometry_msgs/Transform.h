#ifndef _ROS_geometry_msgs_Transform_h
#define _ROS_geometry_msgs_Transform_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

namespace geometry_msgs
{

  class Transform : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _translation_type;
      _translation_type translation;
      typedef geometry_msgs::Quaternion _rotation_type;
      _rotation_type rotation;

    Transform():
      translation(),
      rotation()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->translation.serialize(outbuffer + offset);
      offset += this->rotation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->translation.deserialize(inbuffer + offset);
      offset += this->rotation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Transform"; };
    const char * getMD5(){ return "ac9eff44abf714214112b05d54a3cf9b"; };

  };

}
#endif