#!/usr/bin/env python
import rospy
from champ_msgs.msg import Imu
from champ_msgs.msg import PointArray
from geometry_msgs.msg import Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler
import tf2_ros

class PoseRelay:
    def __init__(self):
        rospy.Subscriber("/champ/imu/raw", Imu, self.imu_callback)
        rospy.Subscriber("/champ/foot/raw", PointArray, self.foot_callback)
    
        self.robot_height = 0

    def foot_callback(self, points):
        self.robot_height = (points.lf.z + points.rf.z + points.lh.z + points.rh.z) / 4

    def imu_callback(self, imu):
        base_broadcaster = tf2_ros.TransformBroadcaster()
        transform_stamped = TransformStamped()

        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "base_footprint"
        transform_stamped.child_frame_id = "base_link"
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 0.0 
        transform_stamped.transform.translation.z = -self.robot_height
        transform_stamped.transform.rotation.x = imu.orientation.x
        transform_stamped.transform.rotation.y = imu.orientation.y
        transform_stamped.transform.rotation.z = imu.orientation.z
        transform_stamped.transform.rotation.w = imu.orientation.w

        base_broadcaster.sendTransform(transform_stamped)

if __name__ == "__main__":
    rospy.init_node('champ_pose_relay', anonymous=True)
    p = PoseRelay()
    rospy.spin()


    