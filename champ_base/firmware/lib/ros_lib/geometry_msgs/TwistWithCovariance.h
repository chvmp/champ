#ifndef _ROS_geometry_msgs_TwistWithCovariance_h
#define _ROS_geometry_msgs_TwistWithCovariance_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Twist.h"

namespace geometry_msgs
{

  class TwistWithCovariance : public ros::Msg
  {
    public:
      typedef geometry_msgs::Twist _twist_type;
      _twist_type twist;
      float covariance[36];

    TwistWithCovariance():
      twist(),
      covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->twist.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 36; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->twist.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 36; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->covariance[i]));
      }
     return offset;
    }

    const char * getType(){ return "geometry_msgs/TwistWithCovariance"; };
    const char * getMD5(){ return "1fe8a28e6890a4cc3ae4c3ca5c7d82e6"; };

  };

}
#endif