
cmake_minimum_required(VERSION 3.5)
project(champ)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
)
set(LIBRARY_NAME "${PROJECT_NAME}_lib")

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${LIBRARY_NAME} INTERFACE
  ${rclcpp_LIBRARIES}
)
# target_include_directories(${PROJECT_NAME} INTERFACE
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>"
# )

# install(TARGETS ${PROJECT_NAME}
#   EXPORT "export_${PROJECT_NAME}"
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION lib
#   INCLUDES DESTINATION include
# )

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


#ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
