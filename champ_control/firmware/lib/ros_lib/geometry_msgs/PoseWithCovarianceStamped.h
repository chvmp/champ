#ifndef _ROS_geometry_msgs_PoseWithCovarianceStamped_h
#define _ROS_geometry_msgs_PoseWithCovarianceStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"

namespace geometry_msgs
{

  class PoseWithCovarianceStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::PoseWithCovariance _pose_type;
      _pose_type pose;

    PoseWithCovarianceStamped():
      header(),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/PoseWithCovarianceStamped"; };
    const char * getMD5(){ return "953b798c0f514ff060a53a3498ce6246"; };

  };

}
#endif