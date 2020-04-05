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

            links_map.push_back("/champ/links_map/left_front");
            links_map.push_back("/champ/links_map/right_front");
            links_map.push_back("/champ/links_map/left_hind");
            links_map.push_back("/champ/links_map/right_hind");

            for(int i = 0; i < 4; i++)
            {
                fillLeg(base.legs[i], nh, model, links_map[i]);
            }
        }
    }
}

#endif