cmake_minimum_required(VERSION 2.8.3)
project(moveit_planner)
## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  geometry_msgs
  moveit_msgs
)

add_service_files(
  FILES
  MovePoint.srv
  MoveQuat.srv
  MovePose.srv
  MoveJoint.srv
  MoveCart.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})

add_executable(main_node src/main.cpp src/moveit_planner.cpp)
add_dependencies(main_node moveit_planner_gencpp)
add_executable(ui_node src/ui_draft.cpp)
add_dependencies(ui_node moveit_planner_gencpp)
add_executable(gripper_node src/gripper_controls.cpp)
add_dependencies(gripper_node moveit_planner_gencpp)
