// Alp Sahin
// User interface node
// Communication between user, moveit_planner main_node services, and gripper_node

// Include header files
#include <iostream>
#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <controller_manager_msgs/SwitchController.h>

// Function Prototypes
void display_menu();
char get_selection();
void switch_controller(ros::ServiceClient &, ros::ServiceClient &);
void reset_world(ros::ServiceClient &);


// Function to display menu options
void display_menu(){
  std::cout << "S - Switch controller" << std::endl;
  std::cout << "R - Reset world" << std::endl;
  std::cout << "Q - Quit" << std::endl;
}

char get_selection(){
  char selection{};
  std::cout << "\nSelect an option: ";
  std::cin >> selection;
  return toupper(selection);
}

void switch_controller(ros::ServiceClient &client){
  controller_manager_msgs::SwitchController srv;
  std::cout << "Select finger to switch (L/R): ";
  char finger{};
  std::cin >> finger;
  finger = toupper(finger);
  std::cout << "Select controller type (P/E): ";
  char controller{};
  std::cin >> controller;
  controller = toupper(controller);
  if (finger=='L'){
    if (controller=='P'){
      srv.request.start_controllers.push_back("l_finger_position");
      srv.request.stop_controllers.push_back("l_finger_effort");
    }
    else{
      srv.request.start_controllers.push_back("l_finger_effort");
      srv.request.stop_controllers.push_back("l_finger_position");
    }
  }
  else{
    if (controller=='P'){
      srv.request.start_controllers.push_back("r_finger_position");
      srv.request.stop_controllers.push_back("r_finger_effort");
    }
    else{
      srv.request.start_controllers.push_back("r_finger_effort");
      srv.request.stop_controllers.push_back("r_finger_position");
    }
  }
  // srv.request.start_controllers.push_back("l_finger_effort");
  // srv.request.stop_controllers.push_back("l_finger_position");
  srv.request.strictness = 2;
  client.call(srv);
  if (srv.response.ok)
    std::cout << "Controller switched successfully." << std::endl;
  else
    std::cout << "Desired controller is already running." << std::endl;
  // std::cout << srv.request.start_controllers << std::endl;
  // srv.request.start_controllers =  current_controllers;
  // srv.request.stop_controllers = desired_controllers;
  // srv.request.strictness = 2;
}

void reset_world(ros::ServiceClient &client){
  std_srvs::Empty srv;
  std::cout << "Resetting world." << std::endl;
  client.call(srv);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "interface");
  ros::NodeHandle interface;
  ros::ServiceClient control_switch = interface.serviceClient<controller_manager_msgs::SwitchController>("gripper/controller_manager/switch_controller");
  ros::ServiceClient reset = interface.serviceClient<std_srvs::Empty>("gazebo/reset_world");

  char selection {};

  do{
    display_menu();
    selection = get_selection();
    switch (selection) {
      case 'S':
        switch_controller(control_switch);
        break;
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
