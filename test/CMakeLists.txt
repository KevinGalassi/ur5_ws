cmake_minimum_required(VERSION 3.0.2)
project(test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  moveit_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  roscpp
  std_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
generate_messages(
  DEPENDENCIES

  geometry_msgs
  moveit_msgs
  std_msgs
)



catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 

  geometry_msgs
  message_runtime
  moveit_msgs 
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  roscpp  
  sensor_msgs
  std_msgs 
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
  moveit_core
  moveit_visual_tools
)

add_executable(test2 src/test2.cpp)
target_link_libraries(test2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})




