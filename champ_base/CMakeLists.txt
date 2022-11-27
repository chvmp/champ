cmake_minimum_required(VERSION 3.5)
project(champ_base)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(champ REQUIRED)
find_package(message_filters REQUIRED)


set(dependencies
  rclcpp
  rclpy
  champ_msgs
  visualization_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  nav_msgs
  tf2_ros
  urdf
  message_filters
  champ
)


# #Download velocity smoother
# set(YOCS_URL "https://github.com/chvmp/yocs_velocity_smoother")
# set(YOCS_DOWNLOAD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../..)
# set(YOCS_DEPENDENCY_PATH ${YOCS_DOWNLOAD_PATH}/yocs_velocity_smoother)

# if (NOT EXISTS "${YOCS_DEPENDENCY_PATH}")
#   message(STATUS "Downloading yocs_velocity_smoother")
#   message(STATUS "${YOCS_DOWNLOAD_PATH}")

#   execute_process(
#     COMMAND git clone ${YOCS_URL} ${YOCS_DEPENDENCY_PATH}
#   )

#   execute_process(
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../..
#     COMMAND catkin_make --pkg yocs_velocity_smoother
#   )
# endif()
# #end of velocity smoother

include_directories(
  include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
  ${champ_INCLUDE_DIRS}/champ/champ/
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME} 
)

add_library(quadruped_controller SHARED src/quadruped_controller.cpp)
ament_target_dependencies(quadruped_controller ${dependencies})
add_executable(quadruped_controller_node src/quadruped_controller_node.cpp)
target_link_libraries(quadruped_controller_node quadruped_controller)
ament_target_dependencies(quadruped_controller_node ${dependencies})


add_library(message_relay SHARED src/message_relay.cpp)
ament_target_dependencies(message_relay ${dependencies})
add_executable(message_relay_node src/message_relay_node.cpp)
target_link_libraries(message_relay_node message_relay)
ament_target_dependencies(message_relay_node ${dependencies})


add_library(state_estimation SHARED src/state_estimation.cpp)
ament_target_dependencies(state_estimation ${dependencies})
add_executable(state_estimation_node src/state_estimation_node.cpp)
target_link_libraries(state_estimation_node state_estimation)
ament_target_dependencies(state_estimation_node ${dependencies})


install(TARGETS 
    quadruped_controller
    quadruped_controller_node
    message_relay
    message_relay_node
    state_estimation
    state_estimation_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


install(TARGETS 
    quadruped_controller_node
    message_relay_node
    state_estimation_node
    DESTINATION lib/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
