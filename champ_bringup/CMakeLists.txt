cmake_minimum_required(VERSION 3.5)
project(champ_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  champ_bringup/joint_calibrator_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()