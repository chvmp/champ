cmake_minimum_required(VERSION 3.5)
project(champ_gazebo)

set(ROS2_CONTROL_URL "https://github.com/rohitmenon86/ros2_controllers")
set(ROS2_CONTROL_DOWNLOAD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(ROS2_CONTROL_DEPENDENCY_PATH ${ROS2_CONTROL_DOWNLOAD_PATH}/ros2_controllers)

if (NOT EXISTS "${ROS2_CONTROL_DEPENDENCY_PATH}")
  message(STATUS "Downloading ROS2_CONTROL_velocity_smoother")
  message(STATUS "${ROS2_CONTROL_DOWNLOAD_PATH}")

  execute_process(
    COMMAND git clone ${ROS2_CONTROL_URL} ${ROS2_CONTROL_DEPENDENCY_PATH} -b $ENV{ROS_DISTRO}
  )

  execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../..
    COMMAND catkin_make --pkg ros2_controllers
  )
endif()

add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(gazebo REQUIRED)

set(dependencies
  rclcpp
  urdf
  champ
  champ_msgs
  gazebo_ros2_control
  gazebo
)

install(PROGRAMS
  scripts/imu_sensor.py
  scripts/odometry_tf.py
  scripts/odometry.py
  DESTINATION bin
)

include_directories(
  include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
  ${champ_INCLUDE_DIRS}/champ/champ/
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(contact_sensor src/contact_sensor.cpp)
target_link_libraries(contact_sensor ${GAZEBO_LIBRARIES})
ament_target_dependencies(contact_sensor ${dependencies})

install(TARGETS 
  contact_sensor
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()