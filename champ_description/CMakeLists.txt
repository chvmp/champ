
cmake_minimum_required(VERSION 2.8.3)
project(champ_description)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/description.launch)
roslaunch_add_file_check(launch/view_urdf.launch)

install(DIRECTORY meshes launch urdf
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
