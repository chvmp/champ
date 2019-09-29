#ifndef _ROS_SERVICE_RequestMessageInfo_h
#define _ROS_SERVICE_RequestMessageInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosserial_msgs
{

static const char REQUESTMESSAGEINFO[] = "rosserial_msgs/RequestMessageInfo";

  class RequestMessageInfoRequest : public ros::Msg
  {
    public:
      typedef const char* _type_type;
      _type_type type;

    RequestMessageInfoRequest():
      type("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_type = strlen(this->type);
      varToArr(outbuffer + offset, length_type);
      offset += 4;
      memcpy(outbuffer + offset, this->type, length_type);
      offset += length_type;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_type;
      arrToVar(length_type, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_type; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_type-1]=0;
      this->type = (char *)(inbuffer + offset-1);
      offset += length_type;
     return offset;
    }

    const char * getType(){ return REQUESTMESSAGEINFO; };
    const char * getMD5(){ return "dc67331de85cf97091b7d45e5c64ab75"; };

  };

  class RequestMessageInfoResponse : public ros::Msg
  {
    public:
      typedef const char* _md5_type;
      _md5_type md5;
      typedef const char* _definition_type;
      _definition_type definition;

    RequestMessageInfoResponse():
      md5(""),
      definition("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_md5 = strlen(this->md5);
      varToArr(outbuffer + offset, length_md5);
      offset += 4;
      memcpy(outbuffer + offset, this->md5, length_md5);
      offset += length_md5;
      uint32_t length_definition = strlen(this->definition);
      varToArr(outbuffer + offset, length_definition);
      offset += 4;
      memcpy(outbuffer + offset, this->definition, length_definition);
      offset += length_definition;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_md5;
      arrToVar(length_md5, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_md5; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_md5-1]=0;
      this->md5 = (char *)(inbuffer + offset-1);
      offset += length_md5;
      uint32_t length_definition;
      arrToVar(length_definition, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_definition; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_definition-1]=0;
      this->definition = (char *)(inbuffer + offset-1);
      offset += length_definition;
     return offset;
    }

    const char * getType(){ return REQUESTMESSAGEINFO; };
    const char * getMD5(){ return "fe452186a069bed40f09b8628fe5eac8"; };

  };

  class RequestMessageInfo {
    public:
    typedef RequestMessageInfoRequest Request;
    typedef RequestMessageInfoResponse Response;
  };

}
#endif
