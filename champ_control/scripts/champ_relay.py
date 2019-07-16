#!/usr/bin/env python
import rospy
from champ_msgs.msg import Point
from champ_msgs.msg import PointArray
from champ_msgs.msg import Joints
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf

class ChampRelay:
    def __init__(self):
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.ranges = []
        self.last_received = rospy.Time.now().to_sec()

        rospy.Subscriber("/champ/foot/raw", PointArray, self.foot_callback)
        rospy.Subscriber("/champ/joint_states/raw", Joints, self.joint_states_callback)

        self.marker_array_pub = rospy.Publisher('/champ/foot', MarkerArray, queue_size = 100)
        self.joint_states_pub = rospy.Publisher('/champ/joint_states', JointState, queue_size = 100)
        self.joint_control_pub = rospy.Publisher('/champ/joint_group_position_controller/command', JointTrajectory, queue_size = 100)

        self.base_broadcaster = tf.TransformBroadcaster()


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

        # joint_states = JointState()
        # joint_states.header.stamp = rospy.Time.now()
        # joint_states.name = [
        #     "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint", 
        #     "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint", 
        #     "lh_hip_joint", "lh_upper_leg_joint","lh_lower_leg_joint", 
        #     "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        # ]

        # joint_states.position = joints.position
        # self.joint_states_pub.publish(joint_states)

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
            "champ_footprint"
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
    v = ChampRelay()
    rospy.spin()


    