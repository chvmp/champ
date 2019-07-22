import pygazebo
import trollius
from trollius import From
import time
import logging
from pygazebo.msg import contacts_pb2
logging.basicConfig()

MOVE = 0
GRAB = 1
RELEASE = 2


@trollius.coroutine
def test():
    print "test"
    manager = yield From(pygazebo.connect(('127.0.0.1', 11345)))

    manager.subscribe('/gazebo/default/physics/contacts','gazebo.msgs.Contacts',contactsCallback)

    publisher = yield From(
                manager.advertise('/gazebo/default/gripperMsg',
                    'gazeboPlugins.msgs.GripperCommand'))                  

                    
    yield From(publisher.wait_for_listener())
    msg = pygazebo.msg.gripperCommand_pb2.GripperCommand()
    msg.cmd = MOVE
    msg.direction.x = 0
    msg.direction.y = 1
    msg.direction.z = 0

def contactsCallback(data):
#    message = pygazebo.msg.gz_string_pb2.GzString.FromString(data)#
    message = contacts_pb2.Contacts.FromString(data)
    print 'Received cntacts message:', str(message.contact.wrench)

#print pygazebo.msg.camera_cmd_pb2
loop = trollius.get_event_loop()
loop.run_until_complete(test())
