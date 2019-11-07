#!/usr/bin/env python
import rospy
import champ_msgs.msg
import sensor_msgs.msg

class IMURelay:
    def __init__(self):
        rospy.Subscriber("/champ/imu/raw", champ_msgs.msg.Imu, self.imu_callback)
        self.imu_pub = rospy.Publisher('/champ/imu/data', sensor_msgs.msg.Imu, queue_size = 100)

    def imu_callback(self, imu):
        imu_msg = sensor_msgs.msg.Imu()
        print imu

if __name__ == "__main__":
    rospy.init_node('champ_imu_relay', anonymous=True)
    i = IMURelay()
    rospy.spin()


    