
cmake_minimum_required(VERSION 2.8.3)
project(champ)

find_package(catkin 
  REQUIRED 
  COMPONENTS
)

catkin_package(
  INCLUDE_DIRS include include/${PROJECT_NAME}
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)