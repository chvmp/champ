/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ROS_MSG_H_
#define _ROS_MSG_H_

#include <stdint.h>
#include <stddef.h>

namespace ros
{

/* Base Message Type */
class Msg
{
public:
  virtual int serialize(unsigned char *outbuffer) const = 0;
  virtual int deserialize(unsigned char *data) = 0;
  virtual const char * getType() = 0;
  virtual const char * getMD5() = 0;

  /**
   * @brief This tricky function handles promoting a 32bit float to a 64bit
   *        double, so that AVR can publish messages containing float64
   *        fields, despite AVV having no native support for double.
   *
   * @param[out] outbuffer pointer for buffer to serialize to.
   * @param[in] f value to serialize.
   *
   * @return number of bytes to advance the buffer pointer.
   *
   */
  static int serializeAvrFloat64(unsigned char* outbuffer, const float f)
  {
    const int32_t* val = (int32_t*) &f;
    int32_t exp = ((*val >> 23) & 255);
    if (exp != 0)
    {
      exp += 1023 - 127;
    }

    int32_t sig = *val;
    *(outbuffer++) = 0;
    *(outbuffer++) = 0;
    *(outbuffer++) = 0;
    *(outbuffer++) = (sig << 5) & 0xff;
    *(outbuffer++) = (sig >> 3) & 0xff;
    *(outbuffer++) = (sig >> 11) & 0xff;
    *(outbuffer++) = ((exp << 4) & 0xF0) | ((sig >> 19) & 0x0F);
    *(outbuffer++) = (exp >> 4) & 0x7F;

    // Mark negative bit as necessary.
    if (f < 0)
    {
      *(outbuffer - 1) |= 0x80;
    }

    return 8;
  }

  /**
   * @brief This tricky function handles demoting a 64bit double to a
   *        32bit float, so that AVR can understand messages containing
   *        float64 fields, despite AVR having no native support for double.
   *
   * @param[in] inbuffer pointer for buffer to deserialize from.
   * @param[out] f pointer to place the deserialized value in.
   *
   * @return number of bytes to advance the buffer pointer.
   */
  static int deserializeAvrFloat64(const unsigned char* inbuffer, float* f)
  {
    uint32_t* val = (uint32_t*)f;
    inbuffer += 3;

    // Copy truncated mantissa.
    *val = ((uint32_t)(*(inbuffer++)) >> 5 & 0x07);
    *val |= ((uint32_t)(*(inbuffer++)) & 0xff) << 3;
    *val |= ((uint32_t)(*(inbuffer++)) & 0xff) << 11;
    *val |= ((uint32_t)(*inbuffer) & 0x0f) << 19;

    // Copy truncated exponent.
    uint32_t exp = ((uint32_t)(*(inbuffer++)) & 0xf0) >> 4;
    exp |= ((uint32_t)(*inbuffer) & 0x7f) << 4;
    if (exp != 0)
    {
      *val |= ((exp) - 1023 + 127) << 23;
    }

    // Copy negative sign.
    *val |= ((uint32_t)(*(inbuffer++)) & 0x80) << 24;

    return 8;
  }

  // Copy data from variable into a byte array
  template<typename A, typename V>
  static void varToArr(A arr, const V var)
  {
    for (size_t i = 0; i < sizeof(V); i++)
      arr[i] = (var >> (8 * i));
  }

  // Copy data from a byte array into variable
  template<typename V, typename A>
  static void arrToVar(V& var, const A arr)
  {
    var = 0;
    for (size_t i = 0; i < sizeof(V); i++)
      var |= (arr[i] << (8 * i));
  }

};

}  // namespace ros

#endif
