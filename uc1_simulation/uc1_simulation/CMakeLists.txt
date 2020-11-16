cmake_minimum_required(VERSION 2.8.3)
project(uc1_simulation)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    moveit_visual_tools
    gazebo_msgs
    message_runtime
    tf tf2 tf2_bullet tf2_ros tf2_geometry_msgs tf2_kdl tf2_msgs
    message_generation
    geometry_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)
find_package(orocos_kdl REQUIRED)

#add_message_files(
#  FILES
#  WorkspaceTrajectoryPoint.msg
#  TrajectoryPoint.msg
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    gazebo_msgs
    message_generation
    message_runtime
  DEPENDS
    EIGEN3
)



###########
## Build ##
###########



include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)
include_directories(/opt/ros/kinetic/include/kdl_conversions/)
include_directories(/opt/ros/kinetic/include/laser_geometry)
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})


#add_executable(my_task_manager_ur5_WMEU src/my_task_manager_ur5_WMEU.cpp src/mytaskmanager.cpp)
#target_link_libraries(my_task_manager_ur5_WMEU ${catkin_LIBRARIES} 
#${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES})

#install(TARGETS code_example DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
