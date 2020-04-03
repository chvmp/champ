#! /usr/bin/env python
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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import tf

class Odom:
    def __init__(self):
        rospy.Subscriber("odom/raw", Odometry, self.odometry_callback) 
        self.odom_broadcaster = tf.TransformBroadcaster()

    def odometry_callback(self, data):
        self.current_linear_speed_x = data.twist.twist.linear.x
        self.current_angular_speed_z = data.twist.twist.angular.z
        
        current_time = rospy.Time.now()

        odom_quat = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )

        self.odom_broadcaster.sendTransform(
            (data.pose.pose.position.x, data.pose.pose.position.y, 0),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

if __name__ == "__main__":
    rospy.init_node("champ_gazebo_odometry_transform", anonymous = True)
    odom = Odom()
    rospy.spin()