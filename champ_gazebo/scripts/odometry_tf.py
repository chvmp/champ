#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import tf

class Odom:
    def __init__(self):
        rospy.Subscriber("odom/ground_truth", Odometry, self.odometry_callback) 
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