#!/usr/bin/env python
import math
import rospy
from champ_msgs.msg import Velocities
from nav_msgs.msg import Odometry
import tf

class OdometryRelay:
    def __init__(self):
        rospy.Subscriber("/champ/velocities/raw", Velocities, self.velocities_callback)
        self.odom_pub = rospy.Publisher('/odom/raw', Odometry, queue_size = 100)
        
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.heading = 0.0
        
        self.prev_time = rospy.Time.now()

    def velocities_callback(self, vel):
        current_time = rospy.Time.now()

        linear_velocity_x = vel.linear_x
        linear_velocity_y = vel.linear_y
        angular_velocity_z = vel.angular_z

        vel_dt = (current_time - self.prev_time).to_sec()

        delta_heading = angular_velocity_z * vel_dt #radians
        delta_x = (linear_velocity_x * math.cos(self.heading) - linear_velocity_y * math.sin(self.heading)) * vel_dt #m
        delta_y = (linear_velocity_x * math.sin(self.heading) + linear_velocity_y * math.cos(self.heading)) * vel_dt #m

        #calculate current position of the robot
        self.x_pos += delta_x
        self.y_pos += delta_y
        self.heading += delta_heading

        #calculate robot's heading in quaternion angle
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.heading)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        #robot's position in x,y, and z
        odom.pose.pose.position.x = self.x_pos
        odom.pose.pose.position.y = self.y_pos
        odom.pose.pose.position.z = 0.0
        #robot's heading in quaternion
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001

        #linear speed from encoders
        odom.twist.twist.linear.x = linear_velocity_x
        odom.twist.twist.linear.y = linear_velocity_y
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        #angular speed from encoders
        odom.twist.twist.angular.z = angular_velocity_z
        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[7] = 0.0001
        odom.twist.covariance[35] = 0.0001

        self.odom_pub.publish(odom)
        
        self.prev_time = current_time

if __name__ == "__main__":
    rospy.init_node('champ_odometry_relay', anonymous=True)
    o = OdometryRelay()
    rospy.spin()
