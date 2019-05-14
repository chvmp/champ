#ifndef _ROS_sensor_msgs_PointCloud_h
#define _ROS_sensor_msgs_PointCloud_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"

namespace sensor_msgs
{

  class PointCloud : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t points_length;
      typedef geometry_msgs::Point32 _points_type;
      _points_type st_points;
      _points_type * points;
      uint32_t channels_length;
      typedef sensor_msgs::ChannelFloat32 _channels_type;
      _channels_type st_channels;
      _channels_type * channels;

    PointCloud():
      header(),
      points_length(0), points(NULL),
      channels_length(0), channels(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->points_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->points_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->points_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->points_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->points_length);
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->channels_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->channels_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->channels_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->channels_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->channels_length);
      for( uint32_t i = 0; i < channels_length; i++){
      offset += this->channels[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t points_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      points_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->points_length);
      if(points_lengthT > points_length)
        this->points = (geometry_msgs::Point32*)realloc(this->points, points_lengthT * sizeof(geometry_msgs::Point32));
      points_length = points_lengthT;
      for( uint32_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(geometry_msgs::Point32));
      }
      uint32_t channels_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      channels_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      channels_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      channels_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->channels_length);
      if(channels_lengthT > channels_length)
        this->channels = (sensor_msgs::ChannelFloat32*)realloc(this->channels, channels_lengthT * sizeof(sensor_msgs::ChannelFloat32));
      channels_length = channels_lengthT;
      for( uint32_t i = 0; i < channels_length; i++){
      offset += this->st_channels.deserialize(inbuffer + offset);
        memcpy( &(this->channels[i]), &(this->st_channels), sizeof(sensor_msgs::ChannelFloat32));
      }
     return offset;
    }

    const char * getType(){ return "sensor_msgs/PointCloud"; };
    const char * getMD5(){ return "d8e9c3f5afbdd8a130fd1d2763945fca"; };

  };

}
#endif