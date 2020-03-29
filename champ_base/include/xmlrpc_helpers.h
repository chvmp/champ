/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Paul Mathieu. */

#ifndef XMLRPCHELPERS_H
#define XMLRPCHELPERS_H

#include <sstream>
#include <ros/ros.h>

namespace xh
{

class XmlrpcHelperException : public ros::Exception
{
    public:
    XmlrpcHelperException(const std::string& what)
        : ros::Exception(what) {}
};

typedef XmlRpc::XmlRpcValue Struct;
typedef XmlRpc::XmlRpcValue Array;

template <class T>
void fetchParam(ros::NodeHandle nh, const std::string& param_name, T& output)
{
    XmlRpc::XmlRpcValue val;
    if (!nh.getParamCached(param_name, val))
    {
        std::ostringstream err_msg;
        err_msg << "could not load parameter '" << param_name << "'. (namespace: "
        << nh.getNamespace() << ")";
        throw XmlrpcHelperException(err_msg.str());
    }

    output = static_cast<T>(val);
}

void checkArrayItem(const Array& col, int index)
{
    if (col.getType() != XmlRpc::XmlRpcValue::TypeArray)
        throw XmlrpcHelperException("not an array");
    if(index >= col.size())
    {
        std::ostringstream err_msg;
        err_msg << "index '" << index << "' is over array capacity";
        throw XmlrpcHelperException(err_msg.str());
    }
}

void checkStructMember(const Struct& col, const std::string& member)
{
    if (col.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        throw XmlrpcHelperException("not a struct");
    if (!col.hasMember(member))
    {
        std::ostringstream err_msg;
        err_msg << "could not find member '" << member << "'";
        throw XmlrpcHelperException(err_msg.str());
    }
}

template <class T>
void getArrayItem(Array& col, int index, T& output) // XXX: XmlRpcValue::operator[] is not const
{
    checkArrayItem(col, index);
    output = static_cast<T>(col[index]);
}

template <class T>
void getStructMember(Struct& col, const std::string& member, T& output)
{
    checkStructMember(col, member);
    output = static_cast<T>(col[member]);
}

} // namespace xh

#endif // XMLRPCHELPERS_H