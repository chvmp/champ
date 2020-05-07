/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef URDF_LOADER_H
#define URDF_LOADER_H

#include "ros/ros.h"
#include <urdf/model.h>
#include <xmlrpc_helpers.h>

#include <quadruped_base/quadruped_base.h>
#include <quadruped_base/quadruped_leg.h>
#include <quadruped_base/quadruped_joint.h>

namespace champ
{
    namespace URDF
    {
        void getPose(urdf::Pose *pose, std::string ref_link, std::string end_link, urdf::Model &model)
        {
            urdf::LinkConstSharedPtr ref_link_ptr = model.getLink(ref_link);

            std::string current_parent_name = end_link;
            urdf::LinkConstSharedPtr prev_link = model.getLink(current_parent_name);

            double roll, pitch, yaw;           

            while(ref_link_ptr->name != current_parent_name)
            {   
                urdf::LinkConstSharedPtr current_link = model.getLink(current_parent_name);
                urdf::Pose current_pose = current_link->parent_joint->parent_to_joint_origin_transform;
              
                current_parent_name = current_link->getParent()->name;
                prev_link = model.getLink(current_parent_name);
                pose->position.x += current_pose.position.x;
                pose->position.y += current_pose.position.y;
                pose->position.z += current_pose.position.z;

                double cur_roll, cur_pitch, cur_yaw;
                current_pose.rotation.getRPY(cur_roll, cur_pitch, cur_yaw);
                roll += cur_roll;
                pitch += cur_pitch;
                yaw += cur_yaw;
            }
            pose->rotation.setFromRPY(roll, pitch, yaw);
        }

        void fillLeg(champ::QuadrupedLeg *leg, ros::NodeHandle nh, urdf::Model &model, std::string links_map)
        {
            xh::Array output;
            xh::fetchParam(nh, links_map, output);
            xh::Struct output_i;

            for(int i = 3; i > -1; i--)
            {
                XmlRpc::XmlRpcValue name_list;
                XmlRpc::XmlRpcValue pos_list;

                std::string ref_link;
                std::string end_link;
                if(i > 0)
                {
                    xh::getArrayItem(output, i-1, ref_link);
                }
                else
                {
                    ref_link = model.getRoot()->name;
                }
                xh::getArrayItem(output, i, end_link);

                urdf::Pose pose;
                getPose(&pose, ref_link, end_link, model);

                double x, y, z, roll, pitch, yaw;
                pose.rotation.getRPY(roll, pitch, yaw);
                x = pose.position.x;
                y = pose.position.y;
                z = pose.position.z;

                leg->joint_chain[i]->setTranslation(x, y, z);
                leg->joint_chain[i]->setRotation(roll, pitch, yaw);
            }
        }

        void loadFromServer(champ::QuadrupedBase &base, ros::NodeHandle nh)
        {
            urdf::Model model;
            if (!model.initParam("robot_description")){
                ROS_ERROR("Failed to parse urdf file");
            } 
            
            ROS_INFO("Successfully parsed urdf file");
            std::vector<std::string> links_map;

            links_map.push_back("links_map/left_front");
            links_map.push_back("links_map/right_front");
            links_map.push_back("links_map/left_hind");
            links_map.push_back("links_map/right_hind");

            for(int i = 0; i < 4; i++)
            {
                fillLeg(base.legs[i], nh, model, links_map[i]);
            }
        }

        std::vector<std::string> getJointNames(ros::NodeHandle nh)
        {
            std::vector<std::string> joints_map;
            std::vector<std::string> joint_names;

            joints_map.push_back("joints_map/left_front");
            joints_map.push_back("joints_map/right_front");
            joints_map.push_back("joints_map/left_hind");
            joints_map.push_back("joints_map/right_hind");

           
            for(int i = 0; i < 4; i++)
            {
                xh::Array output;
                xh::fetchParam(nh, joints_map[i], output);
                xh::Struct output_i;
                for(int j = 0; j < 3; j++)
                {
                    std::string joint_name;
                    xh::getArrayItem(output, j, joint_name);
                    joint_names.push_back(joint_name);
                }
            }

            return joint_names;
        }
    }
}

#endif