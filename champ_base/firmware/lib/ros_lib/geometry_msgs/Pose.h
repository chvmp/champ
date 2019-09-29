#ifndef _ROS_geometry_msgs_Pose_h
#define _ROS_geometry_msgs_Pose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

namespace geometry_msgs
{

  class Pose : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _position_type;
      _position_type position;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;

    Pose():
      position(),
      orientation()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Pose"; };
    const char * getMD5(){ return "e45d45a5a1ce597b249e23fb30fc871f"; };

  };

}
#endif