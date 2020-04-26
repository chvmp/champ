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
)

install(DIRECTORY include/libchamp/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

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
##end of libchamp download

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
  ${LIBCHAMP_HEADERS}
  ${catkin_INCLUDE_DIRS}
)
##end of auto download

add_library(quadruped_controller src/quadruped_controller.cpp)

add_executable(quadruped_controller_node src/quadruped_controller_node.cpp)
target_link_libraries(quadruped_controller_node quadruped_controller ${catkin_LIBRARIES})
add_dependencies(quadruped_controller_node ${catkin_EXPORTED_TARGETS})
add_dependencies(quadruped_controller ${catkin_EXPORTED_TARGETS})

install(PROGRAMS
  scripts/foot_relay.py
  scripts/foot_relay.py
  scripts/imu_relay.py
  scripts/joints_relay.py
  scripts/odometry_relay.py
  scripts/pose_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
    quadruped_controller
    quadruped_controller_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)