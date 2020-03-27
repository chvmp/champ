cmake_minimum_required(VERSION 2.8.3)
project(champ_base)

find_package(catkin 
    REQUIRED 
    COMPONENTS 
    roslaunch
    roscpp
    rospy
    champ_msgs
    visualization_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf
    tf2_ros
)

catkin_package()