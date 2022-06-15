#!/usr/bin/env python3
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

import array
import os
import sys

import rclpy
from champ_msgs.msg import Joints
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointsCalibratorRelay(Node):
    def __init__(self):
        super().__init__('joints_calibrator_relay')
		
        _ = self.create_subscription(JointState, "joints_calibrator", 
                                                                  self.joints_cmd_callback, 1)

        # TODO unhardcode
        joint_controller_topic = "joint_group_effort_controller/joint_trajectory"


        self.joint_minimal_pub = self.create_publisher(Joints, "cmd_joints" ,100)
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory, joint_controller_topic ,100)
        

        joints_map = [None,None,None,None]
        self.declare_parameter('joints_map.left_front',  rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('joints_map.right_front', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('joints_map.left_hind', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('joints_map.right_hind', rclpy.Parameter.Type.STRING_ARRAY)

        joints_map[3] = self.get_parameter('joints_map.left_front').value
        joints_map[2] = self.get_parameter('joints_map.right_front').value
        joints_map[1] = self.get_parameter('joints_map.left_hind').value
        joints_map[0] = self.get_parameter('joints_map.right_hind').value

        self.joint_names = []
        for leg in reversed(joints_map):
            for joint in leg:
                if "foot" not in joint:
                    self.joint_names.append(joint) 

    def joints_cmd_callback(self, joints):
        joint_minimal_msg = Joints()
        for i in range(12):
            joint_minimal_msg.position.append(joints.position[i])

        self.joint_minimal_pub.publish(joint_minimal_msg)

        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=1.0 / 60.0).to_msg()
        point.positions = array.array('d', joint_minimal_msg.position)   
        joint_trajectory_msg.points.append(point)

        self.joint_trajectory_pub.publish(joint_trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    j = JointsCalibratorRelay()

    rclpy.spin(j)


    j.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
