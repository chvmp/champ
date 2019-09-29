#ifndef _ROS_std_msgs_UInt32_h
#define _ROS_std_msgs_UInt32_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace std_msgs
{

  class UInt32 : public ros::Msg
  {
    public:
      typedef uint32_t _data_type;
      _data_type data;

    UInt32():
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->data >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->data >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->data >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->data =  ((uint32_t) (*(inbuffer + offset)));
      this->data |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->data |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return "std_msgs/UInt32"; };
    const char * getMD5(){ return "304a39449588c7f8ce2df6e8001c5fce"; };

  };

}
#endif