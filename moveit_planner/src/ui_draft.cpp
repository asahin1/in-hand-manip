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
#include <moveit_planner.hpp>
#include <std_srvs/Empty.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

// Function Prototypes
void display_menu();
char get_selection();
std::vector<float> get_joint_pos(ros::NodeHandle &);
std::vector<float> get_obj_pos(ros::NodeHandle &);
bool set_pose(ros::ServiceClient &, const float p[]);
bool set_init_pose(ros::ServiceClient &);
void move_up(ros::ServiceClient &);
void move_down(ros::ServiceClient &);
void approach_object(ros::ServiceClient &);
void reset_world(ros::ServiceClient &);
void simulate_1(ros::NodeHandle &, ros::ServiceClient &);
void simulate_2(ros::NodeHandle &, ros::ServiceClient &);
void simulate_3(ros::NodeHandle &, ros::ServiceClient &);
void simulate_4(ros::NodeHandle &, ros::ServiceClient &);


// Function to display menu options
void display_menu(){
  std::cout << "\nJ - Get joint positions" << std::endl;
  std::cout << "O - Get object pose" << std::endl;
  std::cout << "I - Set robot to initial pose" << std::endl;
  std::cout << "U - Move Up" << std::endl;
  std::cout << "D - Move Down" << std::endl;
  std::cout << "A - Approach Object" << std::endl;
  std::cout << "C - Give pose command" << std::endl;
  std::cout << "1 - Simulate 1" << std::endl;
  std::cout << "2 - Simulate 2" << std::endl;
  std::cout << "3 - Simulate 3" << std::endl;
  std::cout << "4 - Simulate 4" << std::endl;
  std::cout << "R - Reset world" << std::endl;
  std::cout << "Q - Quit" << std::endl;
}

char get_selection(){
  char selection{};
  std::cout << "\nSelect an option: ";
  std::cin >> selection;
  return toupper(selection);
}

// Gets joint positions from topic '/panda/joint_states', returns a vector of size 7
std::vector<float> get_joint_pos(ros::NodeHandle &n){
  sensor_msgs::JointState::ConstPtr current_state;
  current_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/panda/joint_states",n);
  std::cout << "Joint position received!" << std::endl;
  std::vector<float> vec;
  for (size_t i{0};i<(current_state->position).size()-2;i++)
    vec.push_back(current_state->position[i+2]);
  return vec;
}

// Gets the pose(position+orientation) of the block object from topic '/gazebo/model_states', returns a vector of size 7
std::vector<float> get_obj_pos(ros::NodeHandle &n){
  gazebo_msgs::ModelStates::ConstPtr model_states;
  model_states = ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states",n);
  std::cout << "Object position received" << std::endl;
  std::vector<float> vec;
  geometry_msgs::Pose obj_pose{model_states->pose[2]};
  vec.push_back(obj_pose.position.x);
  vec.push_back(obj_pose.position.y);
  vec.push_back(obj_pose.position.z);
  vec.push_back(obj_pose.orientation.x);
  vec.push_back(obj_pose.orientation.y);
  vec.push_back(obj_pose.orientation.z);
  vec.push_back(obj_pose.orientation.w);
  return vec;
}

// Gets array of size 7 as input, sets end effector of panda to provided pose
bool set_pose(ros::ServiceClient &client, const float p[]){
  moveit_planner::MovePose srv;
  geometry_msgs::Pose goal;
  goal.position.x = p[0];
  goal.position.y = p[1];
  goal.position.z = p[2];
  goal.orientation.x = p[3];
  goal.orientation.y = p[4];
  goal.orientation.z = p[5];
  goal.orientation.w = p[6];
  srv.request.val = goal;
  srv.request.execute = true;
  client.call(srv);
  return true;
}


bool set_init_pose(ros::ServiceClient &client){
  // for config 1:
  float pose_input []{0.2,0.2,0.35,0.7071068,0,0.7071068,0};
  // for config 2:
  // float pose_input []{0.26,0.2,0.27,0.5,0.5,0.5,0.5};
  return set_pose(client,pose_input);
}

void move_up(ros::ServiceClient &client){
  // for config 1:
  float pose_input []{0.4,0.2,0.4,0.7071068,0,0.7071068,0};
  // for config 2:
  // float pose_input []{0.26,0.2,0.35,0.5,0.5,0.5,0.5};
  set_pose(client,pose_input);
}

void move_down(ros::ServiceClient &client){
  // for config 1:
  float pose_input []{0.4,0.2,0.3,0.7071068,0,0.7071068,0};
  // for config 2:
  // float pose_input []{0.4,0.3,0.5,0.7071068,0,0.7071068,0};
  set_pose(client,pose_input);
}

void approach_object(ros::ServiceClient &client){
  // for config 1:
  float pose_input []{0.4,0.2,0.35,0.7071068,0,0.7071068,0};
  // for config 2:
  // float pose_input []{0.32,0.2,0.27,0.5,0.5,0.5,0.5};
  set_pose(client,pose_input);
}

void simulate_1(ros::NodeHandle &n, ros::ServiceClient &client){
  std::vector<float> obj_pose{get_obj_pos(n)};
  tf::Quaternion q{obj_pose[3],obj_pose[4],obj_pose[5],obj_pose[6]};
  tf::Quaternion rot{0.7071068,0,0,0.7071068};
  tf::Quaternion o1{q*rot};
  tf::Matrix3x3 mat{o1};
  std::vector<float> approach_z{0.2*mat.getColumn(2).getX(),0.2*mat.getColumn(2).getY(),0.2*mat.getColumn(2).getZ()};

  float pose1[] {obj_pose[0]-approach_z[0],obj_pose[1]-approach_z[1],obj_pose[2]-approach_z[2],o1.x(),o1.y(),o1.z(),o1.w()};
  if (set_pose(client,pose1))
    std::cout << "Initialized" << std::endl;
  else
    std::cout << "Failed" << std::endl;

  std::vector<float> approach_z2{0.1*mat.getColumn(2).getX(),0.1*mat.getColumn(2).getY(),0.1*mat.getColumn(2).getZ()};
  float pose2[] {obj_pose[0]-approach_z2[0],obj_pose[1]-approach_z2[1],obj_pose[2]-approach_z2[2],o1.x(),o1.y(),o1.z(),o1.w()};
    if (set_pose(client,pose2))
      std::cout << "Approached" << std::endl;
    else
      std::cout << "Failed" << std::endl;
}

void simulate_2(ros::NodeHandle &n, ros::ServiceClient &client){
  std::vector<float> obj_pose{get_obj_pos(n)};
  tf::Quaternion q{obj_pose[3],obj_pose[4],obj_pose[5],obj_pose[6]};
  tf::Quaternion rot{0.7071068,0,0,0.7071068};
  tf::Quaternion o2{q*rot};
  tf::Quaternion o1{o2*rot};
  tf::Matrix3x3 mat{o1};
  std::vector<float> approach_z{0.2*mat.getColumn(2).getX(),0.2*mat.getColumn(2).getY(),0.2*mat.getColumn(2).getZ()};

  float pose1[] {obj_pose[0]-approach_z[0],obj_pose[1]-approach_z[1],obj_pose[2]-approach_z[2],o1.x(),o1.y(),o1.z(),o1.w()};
  if (set_pose(client,pose1))
    std::cout << "Initialized" << std::endl;
  else
    std::cout << "Failed" << std::endl;

  std::vector<float> approach_z2{0.1*mat.getColumn(2).getX(),0.1*mat.getColumn(2).getY(),0.1*mat.getColumn(2).getZ()};
  float pose2[] {obj_pose[0]-approach_z2[0],obj_pose[1]-approach_z2[1],obj_pose[2]-approach_z2[2],o1.x(),o1.y(),o1.z(),o1.w()};
    if (set_pose(client,pose2))
      std::cout << "Approached" << std::endl;
    else
      std::cout << "Failed" << std::endl;
}

void simulate_3(ros::NodeHandle &n, ros::ServiceClient &client){
  std::vector<float> obj_pose{get_obj_pos(n)};
  tf::Quaternion q{obj_pose[3],obj_pose[4],obj_pose[5],obj_pose[6]};
  tf::Quaternion rot{0.7071068,0,0,0.7071068};
  tf::Quaternion o3{q*rot};
  tf::Quaternion o2{o3*rot};
  tf::Quaternion o1{o2*rot};
  tf::Matrix3x3 mat{o1};
  std::vector<float> approach_z{0.2*mat.getColumn(2).getX(),0.2*mat.getColumn(2).getY(),0.2*mat.getColumn(2).getZ()};

  float pose1[] {obj_pose[0]-approach_z[0],obj_pose[1]-approach_z[1],obj_pose[2]-approach_z[2],o1.x(),o1.y(),o1.z(),o1.w()};
  if (set_pose(client,pose1))
    std::cout << "Initialized" << std::endl;
  else
    std::cout << "Failed" << std::endl;

  std::vector<float> approach_z2{0.1*mat.getColumn(2).getX(),0.1*mat.getColumn(2).getY(),0.1*mat.getColumn(2).getZ()};
  float pose2[] {obj_pose[0]-approach_z2[0],obj_pose[1]-approach_z2[1],obj_pose[2]-approach_z2[2],o1.x(),o1.y(),o1.z(),o1.w()};
    if (set_pose(client,pose2))
      std::cout << "Approached" << std::endl;
    else
      std::cout << "Failed" << std::endl;
}

void simulate_4(ros::NodeHandle &n, ros::ServiceClient &client){
  std::vector<float> obj_pose{get_obj_pos(n)};
  tf::Quaternion q{obj_pose[3],obj_pose[4],obj_pose[5],obj_pose[6]};
  tf::Quaternion rot{0.7071068,0,0,0.7071068};
  tf::Quaternion o1{q};
  tf::Matrix3x3 mat{o1};
  std::vector<float> approach_z{0.2*mat.getColumn(2).getX(),0.2*mat.getColumn(2).getY(),0.2*mat.getColumn(2).getZ()};

  float pose1[] {obj_pose[0]-approach_z[0],obj_pose[1]-approach_z[1],obj_pose[2]-approach_z[2],o1.x(),o1.y(),o1.z(),o1.w()};
  if (set_pose(client,pose1))
    std::cout << "Initialized" << std::endl;
  else
    std::cout << "Failed" << std::endl;

  std::vector<float> approach_z2{0.1*mat.getColumn(2).getX(),0.1*mat.getColumn(2).getY(),0.1*mat.getColumn(2).getZ()};
  float pose2[] {obj_pose[0]-approach_z2[0],obj_pose[1]-approach_z2[1],obj_pose[2]-approach_z2[2],o1.x(),o1.y(),o1.z(),o1.w()};
    if (set_pose(client,pose2))
      std::cout << "Approached" << std::endl;
    else
      std::cout << "Failed" << std::endl;
}

void reset_world(ros::ServiceClient &client){
  std_srvs::Empty srv;
  std::cout << "Resetting world." << std::endl;
  client.call(srv);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "interface");
  ros::NodeHandle interface;
  ros::ServiceClient client_pose = interface.serviceClient<moveit_planner::MovePose>("move_to_pose");
  ros::ServiceClient reset = interface.serviceClient<std_srvs::Empty>("gazebo/reset_world");

  char selection {};

  do{
    display_menu();
    selection = get_selection();
    switch (selection) {
      case 'J':{
        std::vector<float> joint_pos{get_joint_pos(interface)};
        std::cout << "[ ";
        for (auto pos:joint_pos)
          std::cout << pos << " ";
        std::cout << "]" << std::endl;
        break;
      }
      case 'O':{
        std::vector<float> obj_pose{get_obj_pos(interface)};
        std::cout << "Position: ";
        std::cout << "[ " << obj_pose.at(0) << " " << obj_pose.at(1) <<  " " << obj_pose.at(2) << " ]" << std::endl;
        std::cout << "Orientation: ";
        std::cout << "[ " << obj_pose.at(3) << " " << obj_pose.at(4) <<  " " << obj_pose.at(5) << " " << obj_pose.at(6) << " ]" << std::endl;
        break;
      }
      case 'I':{
        std::cout << "Setting robot to initial pose..." << std::endl;
        bool success{set_init_pose(client_pose)};
        if (success)
          std::cout << "Initialization complete." << std::endl;
        else
          std::cout << "Error occured." << std::endl;
        break;
      }
      case 'U':
        move_up(client_pose);
        break;
      case 'D':
        move_down(client_pose);
        break;
      case 'A':
        approach_object(client_pose);
        break;
      case 'C':{
        float target[]{0,0,0,0,0,0,0};
        std::cout << "Waiting for pose input..." << std::endl;
        std::cout << "Position:\nx: " << std::endl;
        std::cin >> target[0];
        std::cout << "y: " << std::endl;
        std::cin >> target[1];
        std::cout << "z: " << std::endl;
        std::cin >> target[2];
        std::cout << "Orientation:\nx: " << std::endl;
        std::cin >> target[3];
        std::cout << "y: " << std::endl;
        std::cin >> target[4];
        std::cout << "z: " << std::endl;
        std::cin >> target[5];
        std::cout << "w: " << std::endl;
        std::cin >> target[6];
        bool success{set_pose(client_pose,target)};
        if (success)
          std::cout << "Pose set." << std::endl;
        else
          std::cout << "Command failed." << std::endl;
        break;
      }
      case '1':{
        simulate_1(interface,client_pose);
        break;
      }
      case '2':{
        simulate_2(interface,client_pose);
        break;
      }
      case '3':{
        simulate_3(interface,client_pose);
        break;
      }
      case '4':{
        simulate_4(interface,client_pose);
        break;
      }
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
