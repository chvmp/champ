#!/usr/bin/env python
import rospy
from champ_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import tf

class PoseRelay:
    def __init__(self):
        rospy.Subscriber("/champ/pose/raw", Pose, self.pose_callback)
        self.base_broadcaster = tf.TransformBroadcaster()

    def pose_callback(self, pose):
        current_time = rospy.Time.now()

        base_orientation = tf.transformations.quaternion_from_euler(pose.roll, pose.pitch, pose.yaw)

        self.base_broadcaster.sendTransform(
            (pose.x, pose.y, pose.z),
            base_orientation,
            current_time,
            "base_link",
            "base_footprint"
        )

if __name__ == "__main__":
    rospy.init_node('champ_pose_relay', anonymous=True)
    p = PoseRelay()
    rospy.spin()


    