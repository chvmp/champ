#ifndef _ROS_sensor_msgs_NavSatFix_h
#define _ROS_sensor_msgs_NavSatFix_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/NavSatStatus.h"

namespace sensor_msgs
{

  class NavSatFix : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef sensor_msgs::NavSatStatus _status_type;
      _status_type status;
      typedef float _latitude_type;
      _latitude_type latitude;
      typedef float _longitude_type;
      _longitude_type longitude;
      typedef float _altitude_type;
      _altitude_type altitude;
      float position_covariance[9];
      typedef uint8_t _position_covariance_type_type;
      _position_covariance_type_type position_covariance_type;
      enum { COVARIANCE_TYPE_UNKNOWN =  0 };
      enum { COVARIANCE_TYPE_APPROXIMATED =  1 };
      enum { COVARIANCE_TYPE_DIAGONAL_KNOWN =  2 };
      enum { COVARIANCE_TYPE_KNOWN =  3 };

    NavSatFix():
      header(),
      status(),
      latitude(0),
      longitude(0),
      altitude(0),
      position_covariance(),
      position_covariance_type(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position_covariance[i]);
      }
      *(outbuffer + offset + 0) = (this->position_covariance_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position_covariance_type);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->position_covariance[i]));
      }
      this->position_covariance_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->position_covariance_type);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/NavSatFix"; };
    const char * getMD5(){ return "2d3a8cd499b9b4a0249fb98fd05cfa48"; };

  };

}
#endif