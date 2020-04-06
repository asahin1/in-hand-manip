cmake_minimum_required(VERSION 2.8.3)
project(gripper_controls)
## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

 find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
 )

 generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tutorials_2
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
 CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_node src/main.cpp)
target_link_libraries(main_node ${catkin_LIBRARIES})
add_dependencies(main_node gripper_generate_messages_cpp)
