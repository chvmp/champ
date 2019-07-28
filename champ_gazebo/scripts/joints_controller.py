#!/usr/bin/env python
import rospy
from champ_msgs.msg import Joints
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointsController:
    def __init__(self):
        rospy.Subscriber("/champ/joint_states/raw", Joints, self.joint_states_callback)
        self.joint_control_pub = rospy.Publisher('/champ/joint_group_position_controller/command', JointTrajectory, queue_size = 100)

    def joint_states_callback(self, joints):
        joint_control = JointTrajectory()
        joint_control.joint_names = [
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint", 
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint", 
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint", 
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        ]
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(1.0 / 60.0)
        point.positions = joints.position    
        joint_control.points.append(point)

        self.joint_control_pub.publish(joint_control)

if __name__ == "__main__":
    rospy.init_node('champ_joints_controller', anonymous=True)
    v = JointsController()
    rospy.spin()


    