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
import rosparam
import os, sys

class JointsRelay:
    def __init__(self):
        rospy.Subscriber("/champ/joint_states/raw", Joints, self.joint_states_callback)
        rospy.Subscriber("/champ/cmd_joints/relay", JointState, self.joints_cmd_callback)

        self.joint_states_pub = rospy.Publisher('/champ/joint_states', JointState, queue_size = 100)
        self.joint_cmd_pub = rospy.Publisher('/champ/cmd_joints', Joints, queue_size = 100)

        self.joint_names = []

        self.in_gazebo = False
        self.in_gazebo = rospy.get_param('/joints_relay/gazebo')

        leg_map = [None,None,None,None]
        leg_map[3] = rospy.get_param('/champ/joints_map/left_front')
        leg_map[2] = rospy.get_param('/champ/joints_map/right_front')
        leg_map[1] = rospy.get_param('/champ/joints_map/left_hind')
        leg_map[0] = rospy.get_param('/champ/joints_map/right_hind')

        for leg in reversed(leg_map):
            for i in range(3):
                self.joint_names.append(leg[i]) 

    def joint_states_callback(self, joints):
        if not self.in_gazebo:
            joint_states = JointState()
            joint_states.header.stamp = rospy.Time.now()
            joint_states.name = self.joint_names

            joint_states.position = joints.position
            self.joint_states_pub.publish(joint_states)

    def joints_cmd_callback(self, joints):
        joint_states_calibrate = Joints()
        
        for i in range(12):
            joint_states_calibrate.position.append(joints.position[i])

        self.joint_cmd_pub.publish(joint_states_calibrate)

if __name__ == "__main__":
    rospy.init_node('champ_joints_relay', anonymous=True)
    j = JointsRelay()
    rospy.spin()