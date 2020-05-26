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

class JointsCalibratorRelay:
    def __init__(self):
        rospy.Subscriber("joints_calibrator", JointState, self.joints_cmd_callback)

        self.joint_cmd_pub = rospy.Publisher('cmd_joints', Joints, queue_size = 100)

    def joints_cmd_callback(self, joints):
        print(joints)
        joint_states_calibrate = Joints()
        
        for i in range(12):
            joint_states_calibrate.position.append(joints.position[i])

        self.joint_cmd_pub.publish(joint_states_calibrate)

if __name__ == "__main__":
    rospy.init_node('joints_calibrator_relay', anonymous=True)
    j = JointsCalibratorRelay()
    rospy.spin()