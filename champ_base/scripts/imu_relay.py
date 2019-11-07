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
        #https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/8
        # ret_val.yaw = atan2(2*(q.kq.real+q.iq.j), q.iq.i - q.jq.j - q.kq.k + q.realq.real);
        # ret_val.pitch = -asin(2*(q.jq.real-q.iq.k)));
        # ret_val.roll = atan2(2*(q.jq.k+q.iq.real), q.iq.i + q.jq.j - q.kq.k - q.realq.real);
if __name__ == "__main__":
    rospy.init_node('champ_imu_relay', anonymous=True)
    i = IMURelay()
    rospy.spin()


    