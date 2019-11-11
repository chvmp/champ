#!/usr/bin/env python
import rospy
import champ_msgs.msg
import sensor_msgs.msg

class IMURelay:
    def __init__(self):
        rospy.Subscriber("/champ/imu/raw", champ_msgs.msg.Imu, self.imu_callback)
        self.imu_pub = rospy.Publisher('/champ/imu/data', sensor_msgs.msg.Imu, queue_size = 100)
        self.mag_pub = rospy.Publisher('/champ/imu/mag', sensor_msgs.msg.MagneticField, queue_size = 100)

    def imu_callback(self, imu):
        imu_data_msg = sensor_msgs.msg.Imu()
        imu_mag_msg = sensor_msgs.msg.MagneticField()

        imu_data_msg.header.stamp = rospy.Time.now()
        imu_data_msg.header.frame_id = "imu_link"

        imu_data_msg.orientation.w = imu.orientation.w
        imu_data_msg.orientation.x = imu.orientation.x
        imu_data_msg.orientation.y = imu.orientation.y
        imu_data_msg.orientation.z = imu.orientation.z

        imu_data_msg.linear_acceleration.x = imu.linear_acceleration.x
        imu_data_msg.linear_acceleration.y = imu.linear_acceleration.y
        imu_data_msg.linear_acceleration.z = imu.linear_acceleration.z

        imu_data_msg.angular_velocity.x = imu.angular_velocity.x
        imu_data_msg.angular_velocity.y = imu.angular_velocity.y
        imu_data_msg.angular_velocity.z = imu.angular_velocity.z

        imu_data_msg.orientation_covariance[0] = 0.0025
        imu_data_msg.orientation_covariance[4] = 0.0025
        imu_data_msg.orientation_covariance[8] = 0.0025

        imu_data_msg.angular_velocity_covariance[0] = 0.000001
        imu_data_msg.angular_velocity_covariance[4] = 0.000001
        imu_data_msg.angular_velocity_covariance[8] = 0.000001

        imu_data_msg.linear_acceleration_covariance[0] = 0.0001
        imu_data_msg.linear_acceleration_covariance[4] = 0.0001
        imu_data_msg.linear_acceleration_covariance[8] = 0.0001

        self.imu_pub.publish(imu_data_msg)

        imu_mag_msg.header.stamp = rospy.Time.now()
        imu_mag_msg.header.frame_id = "imu_link"

        imu_mag_msg.magnetic_field.x = imu.magnetic_field.x
        imu_mag_msg.magnetic_field.y = imu.magnetic_field.y
        imu_mag_msg.magnetic_field.z = imu.magnetic_field.z

        imu_mag_msg.magnetic_field_covariance[0] = 0.000001
        imu_mag_msg.magnetic_field_covariance[4] = 0.000001
        imu_mag_msg.magnetic_field_covariance[8] = 0.000001

        self.mag_pub.publish(imu_mag_msg)

if __name__ == "__main__":
    rospy.init_node('champ_imu_relay', anonymous=True)
    i = IMURelay()
    rospy.spin()


    