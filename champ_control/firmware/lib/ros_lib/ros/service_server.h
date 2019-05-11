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

#ifndef _ROS_SERVICE_SERVER_H_
#define _ROS_SERVICE_SERVER_H_

#include "rosserial_msgs/TopicInfo.h"

#include "ros/publisher.h"
#include "ros/subscriber.h"

namespace ros
{

template<typename MReq , typename MRes, typename ObjT = void>
class ServiceServer : public Subscriber_
{
public:
  typedef void(ObjT::*CallbackT)(const MReq&,  MRes&);

  ServiceServer(const char* topic_name, CallbackT cb, ObjT* obj) :
    pub(topic_name, &resp, rosserial_msgs::TopicInfo::ID_SERVICE_SERVER + rosserial_msgs::TopicInfo::ID_PUBLISHER),
    obj_(obj)
  {
    this->topic_ = topic_name;
    this->cb_ = cb;
  }

  // these refer to the subscriber
  virtual void callback(unsigned char *data)
  {
    req.deserialize(data);
    (obj_->*cb_)(req, resp);
    pub.publish(&resp);
  }
  virtual const char * getMsgType()
  {
    return this->req.getType();
  }
  virtual const char * getMsgMD5()
  {
    return this->req.getMD5();
  }
  virtual int getEndpointType()
  {
    return rosserial_msgs::TopicInfo::ID_SERVICE_SERVER + rosserial_msgs::TopicInfo::ID_SUBSCRIBER;
  }

  MReq req;
  MRes resp;
  Publisher pub;
private:
  CallbackT cb_;
  ObjT* obj_;
};

template<typename MReq , typename MRes>
class ServiceServer<MReq, MRes, void> : public Subscriber_
{
public:
  typedef void(*CallbackT)(const MReq&,  MRes&);

  ServiceServer(const char* topic_name, CallbackT cb) :
    pub(topic_name, &resp, rosserial_msgs::TopicInfo::ID_SERVICE_SERVER + rosserial_msgs::TopicInfo::ID_PUBLISHER)
  {
    this->topic_ = topic_name;
    this->cb_ = cb;
  }

  // these refer to the subscriber
  virtual void callback(unsigned char *data)
  {
    req.deserialize(data);
    cb_(req, resp);
    pub.publish(&resp);
  }
  virtual const char * getMsgType()
  {
    return this->req.getType();
  }
  virtual const char * getMsgMD5()
  {
    return this->req.getMD5();
  }
  virtual int getEndpointType()
  {
    return rosserial_msgs::TopicInfo::ID_SERVICE_SERVER + rosserial_msgs::TopicInfo::ID_SUBSCRIBER;
  }

  MReq req;
  MRes resp;
  Publisher pub;
private:
  CallbackT cb_;
};

}

#endif
