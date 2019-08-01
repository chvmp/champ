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

        self.foot_positions = [[0,0],[0,0],[0,0],[0,0]]
        self.foot_dt = [0,0,0,0]
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.last_contacts_callback_time = 0
        
        now = rospy.Time.now().to_sec()
        self.last_touchdowns = [now,now,now,now]

        self.pos_x = 0
        self.pos_y = 0
        self.theta = 0

    def get_foot_position(self, leg_id):
        if self.tf.frameExists("base_footprint" and self.foot_links[leg_id]) :
            t = self.tf.getLatestCommonTime("base_footprint", self.foot_links[leg_id])
            position, quaternion = self.tf.lookupTransform("base_footprint", self.foot_links[leg_id], t)
            return position
        else:
            return 0,0,0

    def contacts_callback(self, data):
        self.leg_contact_states = data.contacts

    def run(self):
        while not rospy.is_shutdown():
            total_contact = 0
            delta_x_sum = 0
            delta_y_sum = 0
            vx_x_sum = 0
            vx_y_sum = 0 

            leg_contact_states = self.leg_contact_states     
            current_foot_position = [[0,0],[0,0],[0,0],[0,0]]

            for i in range(4):
                current_foot_position[i] = self.get_foot_position(i)
                # print current_foot_position[i]
                if current_foot_position[i] == None:
                    # print "hello"
                    break
            for i in range(4):
                if current_foot_position[i] != None:
                    if self.prev_leg_contact_states and leg_contact_states[i]:

                        delta_x = self.foot_positions[i][0] - current_foot_position[i][0]
                        delta_y = self.foot_positions[i][1] - current_foot_position[i][1]

                        dt = rospy.Time.now().to_sec() - self.last_touchdowns[i]
                        
                        delta_x_sum = delta_x_sum + delta_x
                        delta_y_sum = delta_y_sum + delta_y

                        vx = delta_x / dt
                        vy = delta_y / dt

                        vx_x_sum = vx_x_sum + vx
                        vx_y_sum = vx_y_sum + vy
                        total_contact = total_contact + 1
                        # if delta_y != 0:
                        #     print "x : ", vx_x_sum
                        #     print "y:  ", vx_y_sum

                        self.last_touchdowns[i] = rospy.Time.now().to_sec()
            self.foot_positions= current_foot_position
            self.prev_leg_contact_states = leg_contact_states
            
            mean_vel_x = 0
            mean_vel_y = 0
            if(total_contact > 0):
                mean_vel_x = delta_x_sum 
                mean_vel_y = delta_y_sum

            self.pos_x = self.pos_x + mean_vel_x
            self.pos_y = self.pos_y + mean_vel_y

            current_time = rospy.Time.now()

            odom_quat = quaternion_from_euler (0, 0, 0)

            self.odom_broadcaster.sendTransform(
                (self.pos_x, self.pos_y, 0),
                odom_quat,
                current_time,
                "base_footprint",
                "odom"
            )

            self.last_contacts_callback_time = rospy.Time.now().to_sec()
            rospy.sleep(0.05)

if __name__ == "__main__":
    rospy.init_node("champ_gazebo_odometry", anonymous = True)
    odom = Odometry()
    odom.run()
    # rospy.spin()