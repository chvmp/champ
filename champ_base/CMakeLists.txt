cmake_minimum_required(VERSION 2.8.3)
project(champ_base)

find_package(catkin REQUIRED COMPONENTS
    roslaunch
    roscpp
    rospy
)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/bringup.launch)
