#ifndef _ROS_geometry_msgs_TransformStamped_h
#define _ROS_geometry_msgs_TransformStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Transform.h"

namespace geometry_msgs
{

  class TransformStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _child_frame_id_type;
      _child_frame_id_type child_frame_id;
      typedef geometry_msgs::Transform _transform_type;
      _transform_type transform;

    TransformStamped():
      header(),
      child_frame_id(""),
      transform()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_child_frame_id = strlen(this->child_frame_id);
      varToArr(outbuffer + offset, length_child_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->child_frame_id, length_child_frame_id);
      offset += length_child_frame_id;
      offset += this->transform.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_child_frame_id;
      arrToVar(length_child_frame_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_child_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_child_frame_id-1]=0;
      this->child_frame_id = (char *)(inbuffer + offset-1);
      offset += length_child_frame_id;
      offset += this->transform.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/TransformStamped"; };
    const char * getMD5(){ return "b5764a33bfeb3588febc2682852579b0"; };

  };

}
#endif