#!/usr/bin/env python
import rospy
from champ_msgs.msg import Joints
from sensor_msgs.msg import JointState

class JointsRelay:
    def __init__(self):
        rospy.Subscriber("/champ/joint_states/raw", Joints, self.joint_states_callback)
        self.joint_states_pub = rospy.Publisher('/champ/joint_states', JointState, queue_size = 100)

    def joint_states_callback(self, joints):
        joint_states = JointState()
        joint_states.header.stamp = rospy.Time.now()
        joint_states.name = [
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint", 
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint", 
            "lh_hip_joint", "lh_upper_leg_joint","lh_lower_leg_joint", 
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        ]

        joint_states.position = joints.position
        self.joint_states_pub.publish(joint_states)

if __name__ == "__main__":
    rospy.init_node('champ_joints_relay', anonymous=True)
    j = JointsRelay()
    rospy.spin()


    