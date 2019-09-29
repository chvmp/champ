#ifndef _ROS_sensor_msgs_FluidPressure_h
#define _ROS_sensor_msgs_FluidPressure_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sensor_msgs
{

  class FluidPressure : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _fluid_pressure_type;
      _fluid_pressure_type fluid_pressure;
      typedef float _variance_type;
      _variance_type variance;

    FluidPressure():
      header(),
      fluid_pressure(0),
      variance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->fluid_pressure);
      offset += serializeAvrFloat64(outbuffer + offset, this->variance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->fluid_pressure));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->variance));
     return offset;
    }

    const char * getType(){ return "sensor_msgs/FluidPressure"; };
    const char * getMD5(){ return "804dc5cea1c5306d6a2eb80b9833befe"; };

  };

}
#endif