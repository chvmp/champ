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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointsController:
    def __init__(self):
        rospy.Subscriber("joint_states/raw", Joints, self.joint_states_callback)
        self.joint_control_pub = rospy.Publisher('joint_group_position_controller/command', JointTrajectory, queue_size = 100)

    def joint_states_callback(self, joints):
        joint_control = JointTrajectory()
        joint_control.joint_names = [
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint", 
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint", 
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint", 
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        ]
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(1.0 / 60.0)
        point.positions = joints.position    
        joint_control.points.append(point)

        self.joint_control_pub.publish(joint_control)

if __name__ == "__main__":
    rospy.init_node('champ_gazebo_joints_controller', anonymous=True)
    v = JointsController()
    rospy.spin()


    