cmake_minimum_required(VERSION 3.0.2)
project(offboard_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES offb_node
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
add_executable(${PROJECT_NAME}_node  
   src/waypoint_copy.cpp 
   src/main.cpp
   )
add_executable(velocity_node
   src/velocity_control.cpp
   src/main.cpp
)

add_executable(velocity_test
   src/velocity.cpp
)

add_executable(yaw_node
   src/yaw.cpp
)


#add_executable(tk_land 
#  src/takeoff_land.cpp
#  )

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${EIGEN3_LIBS}
 )

target_link_libraries(velocity_node
 ${catkin_LIBRARIES}
 ${EIGEN3_LIBS}
)
target_link_libraries(velocity_test
 ${catkin_LIBRARIES}
 ${EIGEN3_LIBS}
)

target_link_libraries(yaw_node
 ${catkin_LIBRARIES}
 ${EIGEN3_LIBS}
)