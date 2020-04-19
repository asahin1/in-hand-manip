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

 add_message_files(DIRECTORY msg
   FILES
   Friction.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   PositionCommand.srv
   Holdcommand.srv
   SetControlType.srv
   SetControlValue.srv
   GetFriction.srv
   SetFriction.srv
)


 generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS} include
)

add_executable(main_node src/main.cpp)
target_link_libraries(main_node ${catkin_LIBRARIES})
# add_dependencies(main_node gripper_generate_messages_cpp)

add_executable(high_lvl_ctrl src/high_controller.cpp include/gripper_controls/Hand.cpp)
target_link_libraries(high_lvl_ctrl ${catkin_LIBRARIES})
add_dependencies(high_lvl_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(low_lvl_ctrl src/low_controller.cpp)
target_link_libraries(low_lvl_ctrl ${catkin_LIBRARIES})
add_dependencies(low_lvl_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
