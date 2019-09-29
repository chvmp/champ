#ifndef _ROS_sensor_msgs_RegionOfInterest_h
#define _ROS_sensor_msgs_RegionOfInterest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sensor_msgs
{

  class RegionOfInterest : public ros::Msg
  {
    public:
      typedef uint32_t _x_offset_type;
      _x_offset_type x_offset;
      typedef uint32_t _y_offset_type;
      _y_offset_type y_offset;
      typedef uint32_t _height_type;
      _height_type height;
      typedef uint32_t _width_type;
      _width_type width;
      typedef bool _do_rectify_type;
      _do_rectify_type do_rectify;

    RegionOfInterest():
      x_offset(0),
      y_offset(0),
      height(0),
      width(0),
      do_rectify(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->x_offset >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->x_offset >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->x_offset >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->x_offset >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_offset);
      *(outbuffer + offset + 0) = (this->y_offset >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->y_offset >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->y_offset >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->y_offset >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_offset);
      *(outbuffer + offset + 0) = (this->height >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->height >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->height >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->height >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      *(outbuffer + offset + 0) = (this->width >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->width >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->width >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->width >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      union {
        bool real;
        uint8_t base;
      } u_do_rectify;
      u_do_rectify.real = this->do_rectify;
      *(outbuffer + offset + 0) = (u_do_rectify.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->do_rectify);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->x_offset =  ((uint32_t) (*(inbuffer + offset)));
      this->x_offset |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->x_offset |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->x_offset |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->x_offset);
      this->y_offset =  ((uint32_t) (*(inbuffer + offset)));
      this->y_offset |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->y_offset |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->y_offset |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->y_offset);
      this->height =  ((uint32_t) (*(inbuffer + offset)));
      this->height |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->height |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->height |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->height);
      this->width =  ((uint32_t) (*(inbuffer + offset)));
      this->width |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->width |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->width |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->width);
      union {
        bool real;
        uint8_t base;
      } u_do_rectify;
      u_do_rectify.base = 0;
      u_do_rectify.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->do_rectify = u_do_rectify.real;
      offset += sizeof(this->do_rectify);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/RegionOfInterest"; };
    const char * getMD5(){ return "bdb633039d588fcccb441a4d43ccfe09"; };

  };

}
#endif