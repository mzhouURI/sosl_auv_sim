cmake_minimum_required(VERSION 3.0.2)
project(sosl_auv_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  roslaunch
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
)

add_executable(auv_model src/auv_model.cpp)
target_link_libraries(auv_model ${catkin_LIBRARIES})
add_executable(speed_controller src/speed_controller.cpp)
target_link_libraries(speed_controller ${catkin_LIBRARIES})
add_executable(heading_controller src/heading_controller.cpp)
target_link_libraries(heading_controller ${catkin_LIBRARIES})
