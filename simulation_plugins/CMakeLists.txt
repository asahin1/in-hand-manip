cmake_minimum_required(VERSION 2.8.3)
project(simulation_plugins)

 find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
   gripper_controls
 )
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(friction_plugin SHARED src/friction_plugin.cc)
target_link_libraries(friction_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

## Generate services in the 'srv' folder
add_service_files(
    FILES
    FrictionChange.srv
#   Service1.srv
#   Service2.srv
)


 generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
