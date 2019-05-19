#!/usr/bin/env python

import rospy
from champ_msgs.msg import Point
from champ_msgs.msg import Joints
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import JointState

class Viz:
    def __init__(self):
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.ranges = []
        self.last_received = rospy.Time.now().to_sec()

        rospy.Subscriber("/champ/ee/raw", Point, self.ee_callback)
        rospy.Subscriber("/champ/joint_states/raw", Joints, self.joint_states_callback)

        self.marker_pub = rospy.Publisher('/champ/ee', Marker, queue_size = 10)
        self.joint_states_pub = rospy.Publisher('/champ/joint_states', JointState, queue_size = 10)

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


    def ee_callback(self, point):
        point_marker = Marker()
        point_marker.header.frame_id = "rf_hip_debug_link"
        point_marker.type = Marker.CUBE
        point_marker.action = 0
        point_marker.id = 1

        point_marker.pose.position.x = point.x 
        point_marker.pose.position.y = point.y
        point_marker.pose.position.z = point.z 
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


    