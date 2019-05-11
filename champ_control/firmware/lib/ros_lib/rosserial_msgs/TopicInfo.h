#ifndef _ROS_rosserial_msgs_TopicInfo_h
#define _ROS_rosserial_msgs_TopicInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosserial_msgs
{

  class TopicInfo : public ros::Msg
  {
    public:
      typedef uint16_t _topic_id_type;
      _topic_id_type topic_id;
      typedef const char* _topic_name_type;
      _topic_name_type topic_name;
      typedef const char* _message_type_type;
      _message_type_type message_type;
      typedef const char* _md5sum_type;
      _md5sum_type md5sum;
      typedef int32_t _buffer_size_type;
      _buffer_size_type buffer_size;
      enum { ID_PUBLISHER = 0 };
      enum { ID_SUBSCRIBER = 1 };
      enum { ID_SERVICE_SERVER = 2 };
      enum { ID_SERVICE_CLIENT = 4 };
      enum { ID_PARAMETER_REQUEST = 6 };
      enum { ID_LOG = 7 };
      enum { ID_TIME = 10 };
      enum { ID_TX_STOP = 11 };

    TopicInfo():
      topic_id(0),
      topic_name(""),
      message_type(""),
      md5sum(""),
      buffer_size(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->topic_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->topic_id >> (8 * 1)) & 0xFF;
      offset += sizeof(this->topic_id);
      uint32_t length_topic_name = strlen(this->topic_name);
      varToArr(outbuffer + offset, length_topic_name);
      offset += 4;
      memcpy(outbuffer + offset, this->topic_name, length_topic_name);
      offset += length_topic_name;
      uint32_t length_message_type = strlen(this->message_type);
      varToArr(outbuffer + offset, length_message_type);
      offset += 4;
      memcpy(outbuffer + offset, this->message_type, length_message_type);
      offset += length_message_type;
      uint32_t length_md5sum = strlen(this->md5sum);
      varToArr(outbuffer + offset, length_md5sum);
      offset += 4;
      memcpy(outbuffer + offset, this->md5sum, length_md5sum);
      offset += length_md5sum;
      union {
        int32_t real;
        uint32_t base;
      } u_buffer_size;
      u_buffer_size.real = this->buffer_size;
      *(outbuffer + offset + 0) = (u_buffer_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_buffer_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_buffer_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_buffer_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->buffer_size);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->topic_id =  ((uint16_t) (*(inbuffer + offset)));
      this->topic_id |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->topic_id);
      uint32_t length_topic_name;
      arrToVar(length_topic_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_topic_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_topic_name-1]=0;
      this->topic_name = (char *)(inbuffer + offset-1);
      offset += length_topic_name;
      uint32_t length_message_type;
      arrToVar(length_message_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message_type-1]=0;
      this->message_type = (char *)(inbuffer + offset-1);
      offset += length_message_type;
      uint32_t length_md5sum;
      arrToVar(length_md5sum, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_md5sum; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_md5sum-1]=0;
      this->md5sum = (char *)(inbuffer + offset-1);
      offset += length_md5sum;
      union {
        int32_t real;
        uint32_t base;
      } u_buffer_size;
      u_buffer_size.base = 0;
      u_buffer_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_buffer_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_buffer_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_buffer_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->buffer_size = u_buffer_size.real;
      offset += sizeof(this->buffer_size);
     return offset;
    }

    const char * getType(){ return "rosserial_msgs/TopicInfo"; };
    const char * getMD5(){ return "0ad51f88fc44892f8c10684077646005"; };

  };

}
#endif