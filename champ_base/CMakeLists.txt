cmake_minimum_required(VERSION 2.8.3)
project(champ_base)

add_compile_options(-std=c++11)

find_package(catkin 
    REQUIRED 
    COMPONENTS 
    roscpp
    rospy
    champ_msgs
    visualization_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    nav_msgs
    tf
    tf2_ros
    urdf
    champ
    message_filters
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
    trajectory_msgs
    nav_msgs
    tf
    tf2_ros
    urdf
    message_filters
)

#Download velocity smoother
set(YOCS_URL "https://github.com/chvmp/yocs_velocity_smoother")
set(YOCS_DOWNLOAD_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(YOCS_DEPENDENCY_PATH ${YOCS_DOWNLOAD_PATH}/yocs_velocity_smoother)

if (NOT EXISTS "${YOCS_DEPENDENCY_PATH}")
  message(STATUS "Downloading yocs_velocity_smoother")
  message(STATUS "${YOCS_DOWNLOAD_PATH}")

  execute_process(
    COMMAND git clone ${YOCS_URL} ${YOCS_DEPENDENCY_PATH}
  )

  execute_process(
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../..
    COMMAND catkin_make --pkg yocs_velocity_smoother
  )
endif()
#end of velocity smoother

include_directories(
  include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
  ${champ_INCLUDE_DIRS}/champ/champ/
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_library(quadruped_controller src/quadruped_controller.cpp)
add_library(message_relay src/message_relay.cpp)
add_library(state_estimation src/state_estimation.cpp)

add_executable(quadruped_controller_node src/quadruped_controller_node.cpp)
target_link_libraries(quadruped_controller_node quadruped_controller ${catkin_LIBRARIES})
add_dependencies(quadruped_controller_node ${catkin_EXPORTED_TARGETS})
add_dependencies(quadruped_controller ${catkin_EXPORTED_TARGETS})

add_executable(message_relay_node src/message_relay_node.cpp)
target_link_libraries(message_relay_node message_relay ${catkin_LIBRARIES})
add_dependencies(message_relay_node ${catkin_EXPORTED_TARGETS})
add_dependencies(message_relay ${catkin_EXPORTED_TARGETS})

add_executable(state_estimation_node src/state_estimation_node.cpp)
target_link_libraries(state_estimation_node state_estimation ${catkin_LIBRARIES})
add_dependencies(state_estimation_node ${catkin_EXPORTED_TARGETS})
add_dependencies(state_estimation ${catkin_EXPORTED_TARGETS})

install(TARGETS 
    quadruped_controller
    quadruped_controller_node
    message_relay
    message_relay_node
    state_estimation
    state_estimation_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)