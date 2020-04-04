cmake_minimum_required(VERSION 2.8.3)
project(champ_base)

add_compile_options(-std=c++11)

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
    urdf
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES champ_base
    CATKIN_DEPENDS 
    roscpp
    rospy
    champ_msgs
    visualization_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf
    tf2_ros
    urdf
)

#Download libchamp
set(LIBCHAMP_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/libchamp/include")
set(HEADER_URL "https://github.com/chvmp/libchamp")

if (NOT EXISTS "${LIBCHAMP_HEADERS}")
  message(STATUS "Downloading libchamp...")
  message(STATUS ${CMAKE_CURRENT_SOURCE_DIR}/include)
  execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    COMMAND git clone ${HEADER_URL}
  )
endif()

include_directories(
  include
  ${LIBCHAMP_HEADERS}
  ${catkin_INCLUDE_DIRS}
)
##end of auto download

add_library(quadruped_controller src/quadruped_controller.cpp)

add_executable(quadruped_controller_node src/quadruped_controller_node.cpp)
target_link_libraries(quadruped_controller_node quadruped_controller ${catkin_LIBRARIES})

add_dependencies(quadruped_controller_node champ_msgs_generate_messages_cpp)
