#ifndef _ROS_champ_msgs_Contacts_h
#define _ROS_champ_msgs_Contacts_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace champ_msgs
{

  class Contacts : public ros::Msg
  {
    public:
      uint32_t contacts_length;
      typedef bool _contacts_type;
      _contacts_type st_contacts;
      _contacts_type * contacts;

    Contacts():
      contacts_length(0), contacts(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->contacts_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->contacts_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->contacts_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->contacts_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->contacts_length);
      for( uint32_t i = 0; i < contacts_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_contactsi;
      u_contactsi.real = this->contacts[i];
      *(outbuffer + offset + 0) = (u_contactsi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->contacts[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t contacts_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      contacts_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      contacts_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      contacts_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->contacts_length);
      if(contacts_lengthT > contacts_length)
        this->contacts = (bool*)realloc(this->contacts, contacts_lengthT * sizeof(bool));
      contacts_length = contacts_lengthT;
      for( uint32_t i = 0; i < contacts_length; i++){
      union {
        bool real;
        uint8_t base;
      } u_st_contacts;
      u_st_contacts.base = 0;
      u_st_contacts.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->st_contacts = u_st_contacts.real;
      offset += sizeof(this->st_contacts);
        memcpy( &(this->contacts[i]), &(this->st_contacts), sizeof(bool));
      }
     return offset;
    }

    const char * getType(){ return "champ_msgs/Contacts"; };
    const char * getMD5(){ return "3470d51bc28d5527f9ed97eb122d52f4"; };

  };

}
#endif