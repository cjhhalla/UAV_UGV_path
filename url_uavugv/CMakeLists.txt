cmake_minimum_required(VERSION 3.0.2)
project(url_uavugv)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  mavros_msgs
  std_msgs
  geographic_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/circle_uav.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

