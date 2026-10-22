#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt

# Global variables to store joint state data
joint_positions = []
joint_velocities = []
joint_efforts = []
joint_names = []
time_stamps = []

def joint_states_callback(msg):
    global joint_positions, joint_velocities, joint_efforts, time_stamps
    
    # Store the joint names only once
    if not joint_names:
        joint_names.extend(msg.name)
    
    # Append the data to the corresponding lists
    joint_positions.append(msg.position)
    joint_velocities.append(msg.velocity)
    joint_efforts.append(msg.effort)
    time_stamps.append(rospy.get_time())

def main():
    rospy.init_node('plot_joint_states')

    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    # Spin the node
    rospy.spin()

    # After shutdown, plot the data
    for i, name in enumerate(joint_names):
        plt.figure()
        plt.subplot(3, 1, 1)
        plt.plot(time_stamps, [pos[i] for pos in joint_positions], label=f'Position of {name}')
        plt.xlabel('Time [s]')
        plt.ylabel('Position [rad]')
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(time_stamps, [vel[i] for vel in joint_velocities], label=f'Velocity of {name}')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [rad/s]')
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(time_stamps, [eff[i] for eff in joint_efforts], label=f'Effort of {name}')
        plt.xlabel('Time [s]')
        plt.ylabel('Effort [Nm]')
        plt.legend()

        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
