cmake_minimum_required(VERSION 2.8.3)
project(joy_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  joy
  std_msgs
)

catkin_package()

add_executable(joy_teleop src/joy_teleop.cpp)

target_link_libraries(joy_teleop ${catkin_LIBRARIES})

