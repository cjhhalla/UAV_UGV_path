cmake_minimum_required(VERSION 3.0.2)
project(url_uavugv)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  mavros_msgs
  std_msgs
  geographic_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rect_ugv src/ugv.cpp src/rect_ugv.cpp)
target_link_libraries(rect_ugv ${catkin_LIBRARIES})

add_executable(circle_ugv src/ugv.cpp src/circle_ugv.cpp)
target_link_libraries(circle_ugv ${catkin_LIBRARIES})

add_executable(figure8_ugv src/ugv.cpp src/figure8_ugv.cpp)
target_link_libraries(figure8_ugv ${catkin_LIBRARIES})

add_executable(fail_safe src/fail_safe_sub.cpp)
target_link_libraries(fail_safe ${catkin_LIBRARIES})

add_executable(visual src/visualization.cpp)
target_link_libraries(visual ${catkin_LIBRARIES})