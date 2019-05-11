#ifndef _ROS_SERVICE_RequestParam_h
#define _ROS_SERVICE_RequestParam_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosserial_msgs
{

static const char REQUESTPARAM[] = "rosserial_msgs/RequestParam";

  class RequestParamRequest : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;

    RequestParamRequest():
      name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    const char * getType(){ return REQUESTPARAM; };
    const char * getMD5(){ return "c1f3d28f1b044c871e6eff2e9fc3c667"; };

  };

  class RequestParamResponse : public ros::Msg
  {
    public:
      uint32_t ints_length;
      typedef int32_t _ints_type;
      _ints_type st_ints;
      _ints_type * ints;
      uint32_t floats_length;
      typedef float _floats_type;
      _floats_type st_floats;
      _floats_type * floats;
      uint32_t strings_length;
      typedef char* _strings_type;
      _strings_type st_strings;
      _strings_type * strings;

    RequestParamResponse():
      ints_length(0), ints(NULL),
      floats_length(0), floats(NULL),
      strings_length(0), strings(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->ints_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ints_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ints_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ints_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ints_length);
      for( uint32_t i = 0; i < ints_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_intsi;
      u_intsi.real = this->ints[i];
      *(outbuffer + offset + 0) = (u_intsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_intsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_intsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_intsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ints[i]);
      }
      *(outbuffer + offset + 0) = (this->floats_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->floats_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->floats_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->floats_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->floats_length);
      for( uint32_t i = 0; i < floats_length; i++){
      union {
        float real;
        uint32_t base;
      } u_floatsi;
      u_floatsi.real = this->floats[i];
      *(outbuffer + offset + 0) = (u_floatsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_floatsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_floatsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_floatsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->floats[i]);
      }
      *(outbuffer + offset + 0) = (this->strings_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->strings_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->strings_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->strings_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->strings_length);
      for( uint32_t i = 0; i < strings_length; i++){
      uint32_t length_stringsi = strlen(this->strings[i]);
      varToArr(outbuffer + offset, length_stringsi);
      offset += 4;
      memcpy(outbuffer + offset, this->strings[i], length_stringsi);
      offset += length_stringsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t ints_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      ints_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      ints_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      ints_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->ints_length);
      if(ints_lengthT > ints_length)
        this->ints = (int32_t*)realloc(this->ints, ints_lengthT * sizeof(int32_t));
      ints_length = ints_lengthT;
      for( uint32_t i = 0; i < ints_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_ints;
      u_st_ints.base = 0;
      u_st_ints.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_ints.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_ints.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_ints.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_ints = u_st_ints.real;
      offset += sizeof(this->st_ints);
        memcpy( &(this->ints[i]), &(this->st_ints), sizeof(int32_t));
      }
      uint32_t floats_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      floats_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      floats_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      floats_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->floats_length);
      if(floats_lengthT > floats_length)
        this->floats = (float*)realloc(this->floats, floats_lengthT * sizeof(float));
      floats_length = floats_lengthT;
      for( uint32_t i = 0; i < floats_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_floats;
      u_st_floats.base = 0;
      u_st_floats.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_floats.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_floats.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_floats.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_floats = u_st_floats.real;
      offset += sizeof(this->st_floats);
        memcpy( &(this->floats[i]), &(this->st_floats), sizeof(float));
      }
      uint32_t strings_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      strings_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      strings_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      strings_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->strings_length);
      if(strings_lengthT > strings_length)
        this->strings = (char**)realloc(this->strings, strings_lengthT * sizeof(char*));
      strings_length = strings_lengthT;
      for( uint32_t i = 0; i < strings_length; i++){
      uint32_t length_st_strings;
      arrToVar(length_st_strings, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_strings; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_strings-1]=0;
      this->st_strings = (char *)(inbuffer + offset-1);
      offset += length_st_strings;
        memcpy( &(this->strings[i]), &(this->st_strings), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return REQUESTPARAM; };
    const char * getMD5(){ return "9f0e98bda65981986ddf53afa7a40e49"; };

  };

  class RequestParam {
    public:
    typedef RequestParamRequest Request;
    typedef RequestParamResponse Response;
  };

}
#endif
