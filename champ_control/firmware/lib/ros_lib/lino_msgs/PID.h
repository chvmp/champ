#ifndef _ROS_lino_msgs_PID_h
#define _ROS_lino_msgs_PID_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lino_msgs
{

  class PID : public ros::Msg
  {
    public:
      typedef float _p_type;
      _p_type p;
      typedef float _d_type;
      _d_type d;
      typedef float _i_type;
      _i_type i;

    PID():
      p(0),
      d(0),
      i(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_p;
      u_p.real = this->p;
      *(outbuffer + offset + 0) = (u_p.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p);
      union {
        float real;
        uint32_t base;
      } u_d;
      u_d.real = this->d;
      *(outbuffer + offset + 0) = (u_d.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->d);
      union {
        float real;
        uint32_t base;
      } u_i;
      u_i.real = this->i;
      *(outbuffer + offset + 0) = (u_i.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_p;
      u_p.base = 0;
      u_p.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_p.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_p.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_p.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->p = u_p.real;
      offset += sizeof(this->p);
      union {
        float real;
        uint32_t base;
      } u_d;
      u_d.base = 0;
      u_d.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_d.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_d.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_d.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->d = u_d.real;
      offset += sizeof(this->d);
      union {
        float real;
        uint32_t base;
      } u_i;
      u_i.base = 0;
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->i = u_i.real;
      offset += sizeof(this->i);
     return offset;
    }

    const char * getType(){ return "lino_msgs/PID"; };
    const char * getMD5(){ return "a559df187bdf63f426d5f304b6b28bb4"; };

  };

}
#endif