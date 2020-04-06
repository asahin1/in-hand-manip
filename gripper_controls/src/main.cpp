// Alp Sahin
// User interface node
// Communication between user, moveit_planner main_node services, and gripper_node

// Include header files
#include <iostream>
#include <cstdlib>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>
#include <std_srvs/Empty.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

// Function Prototypes
void display_menu();
char get_selection();
void reset_world(ros::ServiceClient &);


// Function to display menu options
void display_menu(){
  std::cout << "R - Reset world" << std::endl;
  std::cout << "Q - Quit" << std::endl;
}

char get_selection(){
  char selection{};
  std::cout << "\nSelect an option: ";
  std::cin >> selection;
  return toupper(selection);
}


void reset_world(ros::ServiceClient &client){
  std_srvs::Empty srv;
  std::cout << "Resetting world." << std::endl;
  client.call(srv);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "interface");
  ros::NodeHandle interface;
  ros::ServiceClient reset = interface.serviceClient<std_srvs::Empty>("gazebo/reset_world");

  char selection {};

  do{
    display_menu();
    selection = get_selection();
    switch (selection) {
      case 'R':
        reset_world(reset);
        break;
      case 'Q':
        std::cout << "\nGoodbye.." << std::endl;
        break;
      default:
        std::cout << "\nUnknown option." << std::endl;
    }

  }while(selection!='Q');

  return 0;
}
