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
from champ_msgs.msg import Pose

from champ_msgs.msg import PointArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros

class PoseCmd:
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber('champ/cmd_pose', Pose, self.pose_callback)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def pose_callback(self, data):
        self.roll = data.roll
        self.pitch = data.pitch
        self.yaw = data.yaw

class PoseRelay:
    def __init__(self):
        rospy.Subscriber("/champ/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/champ/foot/raw", PointArray, self.foot_callback)
        self.pose_cmd = PoseCmd()

        self.robot_base = rospy.get_param('/champ/links_map/base')
        self.has_imu = rospy.get_param("/pose_relay/has_imu", True)
        self.robot_height = 0
        self.imu_data = Imu()

    def foot_callback(self, points):
        self.robot_height = (points.lf.z + points.rf.z + points.lh.z + points.rh.z) / 4
        base_broadcaster = tf2_ros.TransformBroadcaster()
        transform_stamped = TransformStamped()

        if self.has_imu:
            quaternion = (self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w)
            pose_euler = euler_from_quaternion(quaternion)
            pose_quat = quaternion_from_euler(pose_euler[0], pose_euler[1], 0)
        else:
            pose_quat = quaternion_from_euler(self.pose_cmd.roll, self.pose_cmd.pitch, self.pose_cmd.yaw)

        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "base_footprint"
        transform_stamped.child_frame_id = self.robot_base
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 0.0 
        transform_stamped.transform.translation.z = -self.robot_height
        transform_stamped.transform.rotation.x = pose_quat[0]
        transform_stamped.transform.rotation.y = pose_quat[1]
        transform_stamped.transform.rotation.z = pose_quat[2]
        transform_stamped.transform.rotation.w = pose_quat[3]

        base_broadcaster.sendTransform(transform_stamped)

    def imu_callback(self, imu):
        self.imu_data = imu

if __name__ == "__main__":
    rospy.init_node('champ_pose_relay', anonymous=True)
    p = PoseRelay()
    rospy.spin()