/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "ros/ros.h"
#include <iostream>
#include <champ/utils/urdf_loader.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <boost/algorithm/string.hpp>
#include <champ_msgs/ContactsStamped.h>

class ContactSensor
{
	bool foot_contacts_[4];
	std::vector<std::string> foot_links_;
	ros::Publisher contacts_publisher_;
	gazebo::transport::SubscriberPtr gazebo_sub;

	public:
		ContactSensor(ros::NodeHandle *nh):
			foot_contacts_ {false,false,false,false}
		{
			std::vector<std::string> joint_names;

			joint_names = champ::URDF::getLinkNames(nh);

			foot_links_.push_back(joint_names[2]);
			foot_links_.push_back(joint_names[6]);
			foot_links_.push_back(joint_names[10]);
			foot_links_.push_back(joint_names[14]);

			contacts_publisher_ = nh->advertise<champ_msgs::ContactsStamped>("foot_contacts", 10);
			
			gazebo::client::setup();
			gazebo::transport::NodePtr node(new gazebo::transport::Node());
			node->Init();
			gazebo_sub = node->Subscribe("~/physics/contacts", &ContactSensor::gazeboCallback_, this);
		}

		void gazeboCallback_(ConstContactsPtr &_msg)
		{
			for(size_t i = 0; i < 4; i++)
			{
				foot_contacts_[i] = false;
			}

			for (int i = 0; i < _msg->contact_size(); ++i) 
			{
				std::vector<std::string> results;
				std::string collision = _msg->contact(i).collision1();
				boost::split(results, collision, [](char c){return c == ':';});

				for(size_t j = 0; j < 4; j++)
				{
					if(foot_links_[j] == results[2])
					{
						foot_contacts_[j] = true;
						break;
					}
				}
			}
		}

		void publishContacts()	
		{
			champ_msgs::ContactsStamped contacts_msg;
			contacts_msg.header.stamp = ros::Time::now();
			contacts_msg.contacts.resize(4);

			for(size_t i = 0; i < 4; i++)
			{
				contacts_msg.contacts[i] = foot_contacts_[i];
			}
			
			contacts_publisher_.publish(contacts_msg);
		}
};

void exitHandler(int sig)
{
	gazebo::client::shutdown();
	ros::shutdown();
}

int main(int _argc, char **_argv)
{
	ros::init(_argc, _argv, "contact_sensor");
	ros::NodeHandle nh("");

	signal(SIGINT, exitHandler);

	ContactSensor sensor(&nh);

	ros::Rate contact_publish_rate(50);
 	while (ros::ok())
	{
		sensor.publishContacts();
		ros::spinOnce();

		contact_publish_rate.sleep();
	}
	return 0;
}