#! /usr/bin/env python
import rospy
from champ_msgs.msg import Contacts
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class Odometry:
    def __init__(self):
        rospy.Subscriber("/champ/gazebo/contacts", Contacts, self.contacts_callback)
        self.tf = TransformListener()
        self.leg_contact_states = [False, False, False, False]
        self.prev_leg_contact_states = [False, False, False, False]

        self.foot_links = [
            "lf_foot_link",
            "rf_foot_link",
            "lh_foot_link",
            "rh_foot_link"
        ]

        self.prev_foot_positions = [[0,0],[0,0],[0,0],[0,0]]
        self.prev_theta = [0,0,0,0]
        self.foot_dt = [0,0,0,0]
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        now = rospy.Time.now().to_sec()
        self.last_touchdowns = [now,now,now,now]

        self.pos_x = 0
        self.pos_y = 0
        self.theta = 0

        self.publish_odom_tf(0,0,0,0)
        rospy.sleep(1)

        for i in range(4):
            self.prev_foot_positions[i] = self.get_foot_position(i)
            self.prev_theta[i] = math.atan2(self.prev_foot_positions[i][1], self.prev_foot_positions[i][0])
            
    def publish_odom_tf(self, x, y, z, theta):
        current_time = rospy.Time.now()

        odom_quat = quaternion_from_euler (0, 0, theta)

        self.odom_broadcaster.sendTransform(
            (x, y, z),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

    def get_foot_position(self, leg_id):
        if self.tf.frameExists("base_footprint" and self.foot_links[leg_id]) :
            t = self.tf.getLatestCommonTime("base_footprint", self.foot_links[leg_id])
            position, quaternion = self.tf.lookupTransform("base_footprint", self.foot_links[leg_id], t)
            return position
        else:
            return 0, 0, 0

    def contacts_callback(self, data):
        self.leg_contact_states = data.contacts

    def run(self):
        while not rospy.is_shutdown():
            dt_sum = 0
            leg_contact_states = self.leg_contact_states     
            current_foot_position = [[0,0],[0,0],[0,0],[0,0]]
            current_theta = [0, 0, 0, 0]

            for i in range(4):
                current_foot_position[i] = self.get_foot_position(i)
                current_theta[i] = math.atan2(current_foot_position[i][1], current_foot_position[i][0])

            for i in range(4):
                if current_foot_position[i] != None and leg_contact_states[i]:
                    delta_x = (self.prev_foot_positions[i][0] - current_foot_position[i][0]) / 2
                    delta_y = (self.prev_foot_positions[i][1] - current_foot_position[i][1]) / 2
                    #TODO this value is still wrong
                    delta_theta = (self.prev_theta[i] - current_theta[i])

                    x = delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
                    y = delta_x * math.sin(self.theta) + delta_y * math.cos(self.theta)

                    self.pos_x += x
                    self.pos_y += y
                    self.theta += delta_theta
                    
                    # print i, " ", delta_theta

                    # dt = rospy.Time.now().to_sec() - self.last_touchdowns[i]
                    # dt_sum = dt_sum + dt

                    # self.last_touchdowns[i] = rospy.Time.now().to_sec()

            self.publish_odom_tf(self.pos_x, self.pos_y, 0, self.theta)

            self.prev_foot_positions = current_foot_position
            self.prev_theta = current_theta
            self.prev_leg_contact_states = leg_contact_states
            self.last_contacts_callback_time = rospy.Time.now().to_sec()
            rospy.sleep(0.05)

if __name__ == "__main__":
    rospy.init_node("champ_gazebo_odometry", anonymous = True)
    odom = Odometry()
    odom.run()
