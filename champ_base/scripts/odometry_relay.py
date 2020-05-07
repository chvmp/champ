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

import math
import rospy
from champ_msgs.msg import Velocities
from nav_msgs.msg import Odometry
import tf

class OdometryRelay:
    def __init__(self):
        rospy.Subscriber("velocities/raw", Velocities, self.velocities_callback)
        self.odom_pub = rospy.Publisher('odom/raw', Odometry, queue_size = 100)
        
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.heading = 0.0
        
        self.prev_time = rospy.Time.now()

        self.node_namespace = ""
        self.node_namespace = rospy.get_namespace()
        if len(self.node_namespace) > 1:
            self.node_namespace = self.node_namespace.replace('/', '')
            self.node_namespace += "/"
        else:
            self.node_namespace = ""

        self.odom_frame = self.node_namespace + "odom"
        self.base_footprint_frame = self.node_namespace + "base_footprint"

    def velocities_callback(self, vel):
        current_time = rospy.Time.now()

        linear_velocity_x = vel.linear_x
        linear_velocity_y = vel.linear_y
        angular_velocity_z = vel.angular_z

        vel_dt = (current_time - self.prev_time).to_sec()

        delta_heading = angular_velocity_z * vel_dt #radians
        delta_x = (linear_velocity_x * math.cos(self.heading) - linear_velocity_y * math.sin(self.heading)) * vel_dt #m
        delta_y = (linear_velocity_x * math.sin(self.heading) + linear_velocity_y * math.cos(self.heading)) * vel_dt #m

        #calculate current position of the robot
        self.x_pos += delta_x
        self.y_pos += delta_y
        self.heading += delta_heading

        #calculate robot's heading in quaternion angle
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.heading)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_footprint_frame

        #robot's position in x,y, and z
        odom.pose.pose.position.x = self.x_pos
        odom.pose.pose.position.y = self.y_pos
        odom.pose.pose.position.z = 0.0
        #robot's heading in quaternion
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001

        #linear speed from encoders
        odom.twist.twist.linear.x = linear_velocity_x
        odom.twist.twist.linear.y = linear_velocity_y
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        #angular speed from encoders
        odom.twist.twist.angular.z = angular_velocity_z
        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[7] = 0.0001
        odom.twist.covariance[35] = 0.0001

        self.odom_pub.publish(odom)
        
        self.prev_time = current_time

if __name__ == "__main__":
    rospy.init_node('champ_odometry_relay', anonymous=True)
    o = OdometryRelay()
    rospy.spin()