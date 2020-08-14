#!/usr/bin/env python
'''
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
'''

import rospy
from champ_msgs.msg import Joints
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rosparam
import os, sys

class JointsCalibratorRelay:
    def __init__(self):
        rospy.Subscriber("joints_calibrator", JointState, self.joints_cmd_callback)

        joint_controller_topic = rospy.get_param('champ_controller/joint_controller_topic')
        self.joint_minimal_pub = rospy.Publisher('cmd_joints', Joints, queue_size = 100)
        self.joint_trajectory_pub = rospy.Publisher(joint_controller_topic, JointTrajectory, queue_size = 100)

        joints_map = [None,None,None,None]
        joints_map[3] = rospy.get_param('/joints_map/left_front')
        joints_map[2] = rospy.get_param('/joints_map/right_front')
        joints_map[1] = rospy.get_param('/joints_map/left_hind')
        joints_map[0] = rospy.get_param('/joints_map/right_hind')

        self.joint_names = []
        for leg in reversed(joints_map):
            for joint in leg:
                self.joint_names.append(joint) 

    def joints_cmd_callback(self, joints):
        joint_minimal_msg = Joints()
        for i in range(12):
            joint_minimal_msg.position.append(joints.position[i])

        self.joint_minimal_pub.publish(joint_minimal_msg)

        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(1.0 / 60.0)
        point.positions = joint_minimal_msg.position    
        joint_trajectory_msg.points.append(point)

        self.joint_trajectory_pub.publish(joint_trajectory_msg)


if __name__ == "__main__":
    rospy.init_node('joints_calibrator_relay', anonymous=True)
    j = JointsCalibratorRelay()
    rospy.spin()