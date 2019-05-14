#ifndef _ROS_sensor_msgs_Imu_h
#define _ROS_sensor_msgs_Imu_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

namespace sensor_msgs
{

  class Imu : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Quaternion _orientation_type;
      _orientation_type orientation;
      float orientation_covariance[9];
      typedef geometry_msgs::Vector3 _angular_velocity_type;
      _angular_velocity_type angular_velocity;
      float angular_velocity_covariance[9];
      typedef geometry_msgs::Vector3 _linear_acceleration_type;
      _linear_acceleration_type linear_acceleration;
      float linear_acceleration_covariance[9];

    Imu():
      header(),
      orientation(),
      orientation_covariance(),
      angular_velocity(),
      angular_velocity_covariance(),
      linear_acceleration(),
      linear_acceleration_covariance()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->orientation_covariance[i]);
      }
      offset += this->angular_velocity.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->angular_velocity_covariance[i]);
      }
      offset += this->linear_acceleration.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->linear_acceleration_covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->orientation_covariance[i]));
      }
      offset += this->angular_velocity.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angular_velocity_covariance[i]));
      }
      offset += this->linear_acceleration.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 9; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->linear_acceleration_covariance[i]));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/Imu"; };
    const char * getMD5(){ return "6a62c6daae103f4ff57a132d6f95cec2"; };

  };

}
#endif