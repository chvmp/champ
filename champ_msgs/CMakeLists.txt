cmake_minimum_required(VERSION 3.5)
project(champ_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Velocities.msg"
  "msg/PID.msg"
  "msg/Imu.msg"
  "msg/Point.msg"
  "msg/PointArray.msg"
  "msg/Joints.msg"
  "msg/Contacts.msg"
  "msg/ContactsStamped.msg"
  "msg/Pose.msg"
  DEPENDENCIES std_msgs geometry_msgs
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
