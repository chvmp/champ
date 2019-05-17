#ifndef _ROS_champ_msgs_Joints_h
#define _ROS_champ_msgs_Joints_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace champ_msgs
{

  class Joints : public ros::Msg
  {
    public:
      uint32_t position_length;
      typedef float _position_type;
      _position_type st_position;
      _position_type * position;

    Joints():
      position_length(0), position(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->position_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_length);
      for( uint32_t i = 0; i < position_length; i++){
      union {
        float real;
        uint32_t base;
      } u_positioni;
      u_positioni.real = this->position[i];
      *(outbuffer + offset + 0) = (u_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positioni.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t position_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_length);
      if(position_lengthT > position_length)
        this->position = (float*)realloc(this->position, position_lengthT * sizeof(float));
      position_length = position_lengthT;
      for( uint32_t i = 0; i < position_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_position;
      u_st_position.base = 0;
      u_st_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_position = u_st_position.real;
      offset += sizeof(this->st_position);
        memcpy( &(this->position[i]), &(this->st_position), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "champ_msgs/Joints"; };
    const char * getMD5(){ return "8ec164ae840396df197eeb512c1b8515"; };

  };

}
#endif