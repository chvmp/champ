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
from champ_msgs.msg import Point
from champ_msgs.msg import PointArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

class FootRelay:
    def __init__(self):
        rospy.Subscriber("/champ/foot/raw", PointArray, self.foot_callback)
        self.marker_array_pub = rospy.Publisher('/champ/foot', MarkerArray, queue_size = 100)

        self.robot_base = rospy.get_param('/champ/links_map/base')

    def foot_callback(self, points):
        marker_array = MarkerArray()

        marker_array.markers.append(self.create_marker(points.lf.x, points.lf.y, points.lf.z, 0, self.robot_base))
        marker_array.markers.append(self.create_marker(points.rf.x, points.rf.y, points.rf.z, 1, self.robot_base))
        marker_array.markers.append(self.create_marker(points.lh.x, points.lh.y, points.lh.z, 2, self.robot_base))
        marker_array.markers.append(self.create_marker(points.rh.x, points.rh.y, points.rh.z, 3, self.robot_base))

        self.marker_array_pub.publish(marker_array)

        current_time = rospy.Time.now()

    def create_marker(self, x, y, z, id, frame_id):
        point_marker = Marker()
        point_marker.header.frame_id = frame_id

        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.ADD
        point_marker.id = id
        point_marker.pose.position.x = x
        point_marker.pose.position.y = y
        point_marker.pose.position.z = z
        point_marker.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))
        
        point_marker.scale.x = 0.025
        point_marker.scale.y = 0.025
        point_marker.scale.z = 0.025

        point_marker.color.r = 0.780
        point_marker.color.g = 0.082
        point_marker.color.b = 0.521
        point_marker.color.a = 0.5

        return point_marker

if __name__ == "__main__":
    rospy.init_node('champ_foot_relay', anonymous=True)
    f = FootRelay()
    rospy.spin()