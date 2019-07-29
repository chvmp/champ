#! /usr/bin/env python
import rospy
from champ_msgs.msg import Contacts
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
        self.last_touchdowns = [0,0,0,0]

        self.pos_x = 0
        self.pos_y = 0

    def get_foot_position(self, leg_id):
        
        if self.tf.frameExists(self.foot_links[leg_id]) and self.tf.frameExists("base_footprint"):
            t = self.tf.getLatestCommonTime("base_footprint", self.foot_links[leg_id])
            position, quaternion = self.tf.lookupTransform( "base_footprint", self.foot_links[leg_id], t)
            return position

    def contacts_callback(self, data):
        # print data.contacts

        # total_contact = 0
        # for i in range(4):
        #     current_foot_position = self.get_foot_position(i)
        #     if not self.leg_contact_states[i] and data.contacts[i]:
        #         self.foot_positions[i] = current_foot_position
                
        #         delta_x = current_foot_position[0] - self.foot_positions[i][0]
        #         delta_y = current_foot_position[1] - self.foot_positions[i][1]
        #         total_contact = total_contact + 1

        #         x_sum = x_sum + delta_x
        #         y_sum = y_sum + delta_y
        #         print "hello"

        # if total_contact > 0:
        #     mean_x = x_sum / total_contact
        #     mean_y = y_sum / total_contact
            
        #     dt = rospy.Time.now().to_sec() - self.last_contacts_callback_time
        #     print(mean_x / dt)

        # self.last_contacts_callback_time = rospy.Time.now().to_sec()
        self.leg_contact_states = data.contacts

    def run(self):
        while not rospy.is_shutdown():
            total_contact = 0
            x_sum = 0
            y_sum = 0
            for i in range(4):
                current_foot_position = self.get_foot_position(i)
                if current_foot_position != None:

                    if not self.prev_leg_contact_states[i] and self.leg_contact_states[i]:
                        self.foot_positions[i] = current_foot_position
                        self.last_touchdowns[i] = rospy.Time.now().to_sec()

                    elif self.prev_leg_contact_states[i] and not self.leg_contact_states[i]:
                        delta_x = current_foot_position[0] - self.foot_positions[i][0]
                        delta_y = current_foot_position[1] - self.foot_positions[i][1]

                        total_contact = total_contact + 1

                        dt = rospy.Time.now().to_sec() - self.last_touchdowns[i]

                        vel_x = delta_x / dt
                        vel_y = delta_y / dt
                        
                        x_sum = x_sum + delta_x
                        y_sum = y_sum + delta_y

                    self.prev_leg_contact_states[i] = self.leg_contact_states[i]

            # if total_contact > 0:
            mean_vel_x = x_sum / 4
            mean_vel_y = y_sum / 4
            self.pos_x = self.pos_x + mean_vel_x
            self.pos_y = self.pos_y + mean_vel_y

            current_time = rospy.Time.now()

            odom_quat = quaternion_from_euler (0, 0,0)

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