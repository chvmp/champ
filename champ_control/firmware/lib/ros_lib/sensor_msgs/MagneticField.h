#ifndef _ROS_sensor_msgs_MagneticField_h
#define _ROS_sensor_msgs_MagneticField_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace sensor_msgs
{

  class MagneticField : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Vector3 _magnetic_field_type;
      _magnetic_field_type magnetic_field;
      float magnetic_field_covariance[9];

    MagneticField():
      header(),
      magnetic_field(),
      magnetic_field_covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->magnetic_field.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->magnetic_field_covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->magnetic_field.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->magnetic_field_covariance[i]));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/MagneticField"; };
    const char * getMD5(){ return "2f3b0b43eed0c9501de0fa3ff89a45aa"; };

  };

}
#endif