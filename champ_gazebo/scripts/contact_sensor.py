import pygazebo
import trollius
from trollius import From
import time
from pygazebo.msg import contacts_pb2 #forked version of pygazebo https://github.com/Rimabo/pygazebo

def contactsCallback(data):
    message = contacts_pb2.Contacts.FromString(data)
    if message.contact:
        for contact in message.contact:
            leg = contact.wrench[0].body_1_name.split("::")[1]
            if leg == "rf_lower_leg_link":
                print "rf"
            elif leg == "lh_lower_leg_link":
                print "lf"
            elif leg == "lf_lower_leg_link":
                print "lf"
            elif leg == "rh_lower_leg_link":
                print "rh"

def test():
    print "test"
    manager = yield From(pygazebo.connect(('127.0.0.1', 11345)))

    subscriber = manager.subscribe('/gazebo/default/physics/contacts','gazebo.msgs.Contacts',contactsCallback)

    publisher = yield From(
                manager.advertise('/gazebo/default/gripperMsg',
                    'gazeboPlugins.msgs.GripperCommand'))                  

    while True:
        yield From(trollius.sleep(0.2))
                    
loop = trollius.get_event_loop()
loop.run_until_complete(test())
