#ifndef _ROS_std_msgs_Float64_h
#define _ROS_std_msgs_Float64_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace std_msgs
{

  class Float64 : public ros::Msg
  {
    public:
      typedef float _data_type;
      _data_type data;

    Float64():
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->data));
     return offset;
    }

    const char * getType(){ return "std_msgs/Float64"; };
    const char * getMD5(){ return "fdb28210bfa9d7c91146260178d9a584"; };

  };

}
#endif