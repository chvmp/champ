#!/usr/bin/env python
import rospy
from champ_msgs.msg import Point
from champ_msgs.msg import PointArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import tf

class FootRelay:
    def __init__(self):
        rospy.Subscriber("/champ/foot/raw", PointArray, self.foot_callback)
        self.marker_array_pub = rospy.Publisher('/champ/foot', MarkerArray, queue_size = 100)
        self.base_broadcaster = tf.TransformBroadcaster()

    def foot_callback(self, points):
        marker_array = MarkerArray()
        marker_array.markers.append(self.create_marker(points.lf.x, points.lf.y, points.lf.z, 0, "lf_hip_debug_link"))
        marker_array.markers.append(self.create_marker(points.rf.x, points.rf.y, points.rf.z, 1, "rf_hip_debug_link"))
        marker_array.markers.append(self.create_marker(points.lh.x, points.lh.y, points.lh.z, 2, "lh_hip_debug_link"))
        marker_array.markers.append(self.create_marker(points.rh.x, points.rh.y, points.rh.z, 3, "rh_hip_debug_link"))
        # marker_array.markers.append(self.create_marker(points.lf.x, points.lf.y, points.lf.z, 0, "base_link"))
        # marker_array.markers.append(self.create_marker(points.rf.x, points.rf.y, points.rf.z, 1, "base_link"))
        # marker_array.markers.append(self.create_marker(points.lh.x, points.lh.y, points.lh.z, 2, "base_link"))
        # marker_array.markers.append(self.create_marker(points.rh.x, points.rh.y, points.rh.z, 3, "base_link"))
        self.marker_array_pub.publish(marker_array)

        current_time = rospy.Time.now()

        base_orientation = tf.transformations.quaternion_from_euler(0, 0, 0)

        self.base_broadcaster.sendTransform(
            (0, 0, points.lf.x),
            base_orientation,
            current_time,
            "base_link",
            "base_footprint"
        )

    def create_marker(self, x, y, z, id, frame_id):
        point_marker = Marker()
        point_marker.header.frame_id = frame_id
        point_marker.header.stamp = rospy.Time.now()
        point_marker.lifetime = rospy.Duration.from_sec(10)

        point_marker.type = Marker.SPHERE
        point_marker.action = Marker.MODIFY
        point_marker.id = id
        point_marker.pose.position.x = x
        point_marker.pose.position.y = y
        point_marker.pose.position.z = z
        point_marker.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))
        
        point_marker.scale.x = 0.018
        point_marker.scale.y = 0.018
        point_marker.scale.z = 0.018

        point_marker.color.r = 0.780
        point_marker.color.g = 0.082
        point_marker.color.b = 0.521
        point_marker.color.a = 0.5

        return point_marker

if __name__ == "__main__":
    rospy.init_node('clustering', anonymous=True)
    v = FootRelay()
    rospy.spin()


    