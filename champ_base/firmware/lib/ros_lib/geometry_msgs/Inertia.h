#ifndef _ROS_geometry_msgs_Inertia_h
#define _ROS_geometry_msgs_Inertia_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace geometry_msgs
{

  class Inertia : public ros::Msg
  {
    public:
      typedef float _m_type;
      _m_type m;
      typedef geometry_msgs::Vector3 _com_type;
      _com_type com;
      typedef float _ixx_type;
      _ixx_type ixx;
      typedef float _ixy_type;
      _ixy_type ixy;
      typedef float _ixz_type;
      _ixz_type ixz;
      typedef float _iyy_type;
      _iyy_type iyy;
      typedef float _iyz_type;
      _iyz_type iyz;
      typedef float _izz_type;
      _izz_type izz;

    Inertia():
      m(0),
      com(),
      ixx(0),
      ixy(0),
      ixz(0),
      iyy(0),
      iyz(0),
      izz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->m);
      offset += this->com.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->ixx);
      offset += serializeAvrFloat64(outbuffer + offset, this->ixy);
      offset += serializeAvrFloat64(outbuffer + offset, this->ixz);
      offset += serializeAvrFloat64(outbuffer + offset, this->iyy);
      offset += serializeAvrFloat64(outbuffer + offset, this->iyz);
      offset += serializeAvrFloat64(outbuffer + offset, this->izz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m));
      offset += this->com.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ixx));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ixy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->ixz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->iyy));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->iyz));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->izz));
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Inertia"; };
    const char * getMD5(){ return "1d26e4bb6c83ff141c5cf0d883c2b0fe"; };

  };

}
#endif