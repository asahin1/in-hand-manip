#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

void display_menu();
char get_selection();
void get_joint_pos(ros::NodeHandle &);

void display_menu(){
  std::cout << "\nP - Print current joint positions" << std::endl;
  std::cout << "C - Give pose command" << std::endl;
  std::cout << "Q - Quit" << std::endl;
}

char get_selection(){
  char selection{};
  std::cout << "\nSelect an option: ";
  std::cin >> selection;
  return toupper(selection);
}

void get_joint_pos(ros::NodeHandle &n){
  sensor_msgs::JointState::ConstPtr current_state;
  current_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/panda/joint_states",n);
  std::cout << "Position received!" << std::endl;
  std::cout << current_state << std::endl;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "interface");
  ros::NodeHandle interface;


  char selection {};

  do{
    display_menu();
    selection = get_selection();
    switch (selection) {
      case 'P':
        std::cout << "\nJoint pos to be printed here" << std::endl;
        get_joint_pos(interface);
        break;
      case 'C':
        std::cout << "\nCommand under construction" << std::endl;
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
