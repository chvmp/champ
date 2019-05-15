#!/usr/bin/env python

import rospy
from lino_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

class Viz:
    def __init__(self):
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.ranges = []
        self.last_received = rospy.Time.now().to_sec()
        rospy.Subscriber("point_debug", Point, self.debug_callback)
        self.marker_pub = rospy.Publisher('ee_debug', Marker, queue_size = 10)

    def debug_callback(self, data):
        point_marker = Marker()
        point_marker.header.frame_id = "rf_hip_debug_link"
        point_marker.type = Marker.CUBE
        point_marker.action = 0
        point_marker.id = 1

        point_marker.pose.position.x = data.point.x 
        point_marker.pose.position.y = data.point.y
        point_marker.pose.position.z = data.point.z 
        point_marker.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))
        
        point_marker.scale.x = (0.025); 
        point_marker.scale.y = (0.025); 
        point_marker.scale.z = (0.025); 

        point_marker.color.r = 0.780; 
        point_marker.color.g = 0.082; 
        point_marker.color.b = 0.521; 
        point_marker.color.a = 0.5; 

        self.marker_pub.publish(point_marker)

if __name__ == "__main__":
    rospy.init_node('clustering', anonymous=True)
    v = Viz()
    rospy.spin()


    