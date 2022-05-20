
cmake_minimum_required(VERSION 3.5)
project(champ_gazebo)

add_compile_options(-std=c++14)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(gazebo_ros2_control REQUIRED)


set(dependencies
  rclcpp
  urdf
  champ
  champ_msgs
  gazebo_ros2_control
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


# add_executable(contact_sensor src/contact_sensor.cpp)
# target_link_libraries(contact_sensor ${GAZEBO_LIBRARIES} pthread ${dependencies} )
# ament_target_dependencies(contact_sensor ${dependencies})

# install(TARGETS 
#     contact_sensor
#     ARCHIVE DESTINATION lib
#     LIBRARY DESTINATION lib
#     RUNTIME DESTINATION bin
# )


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})



ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()