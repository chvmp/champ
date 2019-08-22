#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from champ_msgs.msg import Pose
import tf

class SimPose:
    def __init__(self):
        rospy.Subscriber("odom/ground_truth", Odometry, self.odometry_callback) 
        self.sim_pose_publisher = rospy.Publisher("/champ/gazebo/pose", Pose, queue_size=50)

    def odometry_callback(self, data):
        sim_pose_msg = Pose()
        sim_pose_msg.roll = data.pose.pose.orientation.x
        sim_pose_msg.pitch = data.pose.pose.orientation.y
        sim_pose_msg.yaw = data.pose.pose.orientation.z

        self.sim_pose_publisher.publish(sim_pose_msg)

if __name__ == "__main__":
    rospy.init_node("champ_gazebo_sim_pose", anonymous = True)
    odom = SimPose()
    rospy.spin()