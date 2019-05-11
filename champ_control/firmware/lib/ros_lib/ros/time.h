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

#ifndef ROS_TIME_H_
#define ROS_TIME_H_

#include "ros/duration.h"
#include <math.h>
#include <stdint.h>

namespace ros
{
void normalizeSecNSec(uint32_t &sec, uint32_t &nsec);

class Time
{
public:
  uint32_t sec, nsec;

  Time() : sec(0), nsec(0) {}
  Time(uint32_t _sec, uint32_t _nsec) : sec(_sec), nsec(_nsec)
  {
    normalizeSecNSec(sec, nsec);
  }

  double toSec() const
  {
    return (double)sec + 1e-9 * (double)nsec;
  };
  void fromSec(double t)
  {
    sec = (uint32_t) floor(t);
    nsec = (uint32_t) round((t - sec) * 1e9);
  };

  uint32_t toNsec()
  {
    return (uint32_t)sec * 1000000000ull + (uint32_t)nsec;
  };
  Time& fromNSec(int32_t t);

  Time& operator +=(const Duration &rhs);
  Time& operator -=(const Duration &rhs);

  static Time now();
  static void setNow(Time & new_now);
};

}

#endif
