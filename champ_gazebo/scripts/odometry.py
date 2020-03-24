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
from champ_msgs.msg import Contacts
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose, Twist, Vector3
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry

import math

class ChampOdometry:
    def __init__(self):
        rospy.Subscriber("/champ/gazebo/contacts", Contacts, self.contacts_callback)
        self.odom_publisher = rospy.Publisher("odom/raw", Odometry, queue_size=50)

        self.odom_broadcaster = tf.TransformBroadcaster()
        self.tf = TransformListener()

        self.foot_links = [
            "lf_foot_link",
            "rf_foot_link",
            "lh_foot_link",
            "rh_foot_link"
        ]

        self.nominal_foot_positions = [[0,0],[0,0],[0,0],[0,0]]
        self.prev_foot_positions = [[0,0],[0,0],[0,0],[0,0]]
        self.prev_theta = [0,0,0,0]
        self.prev_stance_angles = [0,0,0,0]
        self.prev_time = 0

        self.pos_x = 0
        self.pos_y = 0
        self.theta = 0

        self.publish_odom_tf(0,0,0,0)
        rospy.sleep(1)

        for i in range(4):
            self.nominal_foot_positions[i] = self.get_foot_position(i)
            self.prev_foot_positions[i] = self.nominal_foot_positions[i]
            self.prev_theta[i] = math.atan2(self.prev_foot_positions[i][1], self.prev_foot_positions[i][0])

    def publish_odom(self, x, y, z, theta, vx, vy, vth):
        current_time = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose = Pose(Point(x, y, z), Quaternion(*odom_quat))

        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        self.odom_publisher.publish(odom)

    def publish_odom_tf(self, x, y, z, theta):
        current_time = rospy.Time.now()

        odom_quat = quaternion_from_euler (0, 0, theta)

        self.odom_broadcaster.sendTransform(
            (x, y, z),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

    def get_foot_position(self, leg_id):
        if self.tf.frameExists("base_link" and self.foot_links[leg_id]) :
            t = self.tf.getLatestCommonTime("base_link", self.foot_links[leg_id])
            position, quaternion = self.tf.lookupTransform("base_link", self.foot_links[leg_id], t)
            return position
        else:
            return 0, 0, 0

    def contacts_callback(self, data):
        self.leg_contact_states = data.contacts

    def is_almost_equal(self, a, b, max_rel_diff):
        diff = abs(a - b)
        a = abs(a)
        b = abs(b)
        largest = 0
        if b > a:
            largest = b
        else:
            larget = a

        if diff <= (largest * max_rel_diff):
            return True

        return False

    def run(self):
        while not rospy.is_shutdown():
            leg_contact_states = self.leg_contact_states     
            current_foot_position = [[0,0],[0,0],[0,0],[0,0]]
            stance_angles = [0, 0, 0, 0]
            current_theta = [0, 0, 0, 0]
            delta_theta = 0
            in_xy = False
            total_contact = sum(leg_contact_states)
            x_sum = 0
            y_sum = 0
            theta_sum = 0
   
            for i in range(4):
                current_foot_position[i] = self.get_foot_position(i)

            for i in range(4):
                current_theta[i] = math.atan2(current_foot_position[i][1], current_foot_position[i][0])
                from_nominal_x = self.nominal_foot_positions[i][0] - current_foot_position[i][0]
                from_nominal_y = self.nominal_foot_positions[i][1] - current_foot_position[i][1]
                stance_angles[i] = math.atan2(from_nominal_y, from_nominal_x)
                # print stance_angles
                #check if it's moving in the x or y axis
                if self.is_almost_equal(stance_angles[i], abs(1.5708), 0.001) or self.is_almost_equal(stance_angles[i], abs(3.1416), 0.001):
                    in_xy = True
                
                if current_foot_position[i] != None and leg_contact_states[i] == True and total_contact == 2:
                    delta_x = (self.prev_foot_positions[i][0] - current_foot_position[i][0]) / 2
                    delta_y = (self.prev_foot_positions[i][1] - current_foot_position[i][1]) / 2
    
                    x = delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
                    y = delta_x * math.sin(self.theta) + delta_y * math.cos(self.theta)

                    x_sum += delta_x
                    y_sum += delta_y

                    self.pos_x += x
                    self.pos_y += y

                    if not in_xy:
                        delta_theta = self.prev_theta[i] - current_theta[i]
                        theta_sum += delta_theta
                        self.theta += delta_theta / 2

            now = rospy.Time.now().to_sec() 
            dt = now - self.prev_time

            vx = x_sum / dt
            vy = y_sum / dt
            vth = theta_sum / dt

            self.publish_odom(self.pos_x, self.pos_y, 0, self.theta, vx, vy, vth)

            # self.publish_odom_tf(self.pos_x, self.pos_y, 0, self.theta)

            self.prev_foot_positions = current_foot_position
            self.prev_theta = current_theta
            self.prev_stance_angles = stance_angles

            self.prev_time = now
            rospy.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node("champ_gazebo_odometry", anonymous = True)
    odom = ChampOdometry()
    odom.run()
