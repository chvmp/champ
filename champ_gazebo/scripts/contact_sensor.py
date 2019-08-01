#!/usr/bin/env python
import rospy
from champ_msgs.msg import Contacts
import pygazebo
import trollius
from trollius import From
import time
from pygazebo.msg import contacts_pb2 #forked version of pygazebo https://github.com/Rimabo/pygazebo

class ContactSensor:
    def __init__(self):
        self.contacts_publisher = rospy.Publisher('/champ/gazebo/contacts', Contacts, queue_size = 1)

    def contactsCallback(self, data):
        message = contacts_pb2.Contacts.FromString(data)
        leg_contacts = [0,0,0,0]
        if message.contact:
            for contact in message.contact:
                leg = contact.wrench[0].body_1_name.split("::")[1]
                if leg == "rf_lower_leg_link":
                    leg_contacts[1] = 1
                elif leg == "lh_lower_leg_link":
                    leg_contacts[2] = 1
                elif leg == "lf_lower_leg_link":
                    leg_contacts[0] = 1
                elif leg == "rh_lower_leg_link":
                    leg_contacts[3] = 1
                    
        leg_contact_msg = Contacts()
        leg_contact_msg.contacts = leg_contacts
        self.contacts_publisher.publish(leg_contact_msg)

    def run(self):
        gazebo_manager = yield From(pygazebo.connect(('127.0.0.1', 11345)))
        contacts_subscriber = gazebo_manager.subscribe('/gazebo/default/physics/contacts','gazebo.msgs.Contacts', self.contactsCallback)

        while not rospy.is_shutdown():
            yield From(trollius.sleep(0.09))

if __name__ == "__main__":
    rospy.init_node("champ_gazebo_contact_sensor", anonymous = True)
    contact_sensor = ContactSensor()

    loop = trollius.get_event_loop()
    loop.run_until_complete(contact_sensor.run())
