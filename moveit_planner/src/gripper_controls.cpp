//Gripper controller
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>


int main(int argc, char **argv) {

  ros::init(argc, argv, "gripper");

  ros::NodeHandle n;

  ros::Publisher gripper1_pub = n.advertise<std_msgs::Float64>("/panda/panda_hand_controller1/command", 1000);
  ros::Publisher gripper2_pub = n.advertise<std_msgs::Float64>("/panda/panda_hand_controller2/command", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok()){

    char selection{};
    std::cout << "C - Close\nO - Open\nQ - Quit" << std::endl;
    std::cin >> selection;

    std_msgs::Float64 input1;
    std_msgs::Float64 input2;

    float finger_pos[]{0,0};
    if (toupper(selection) == 'C'){
      input1.data = 0.01;
      input2.data = 0.01;
    }
    else if (toupper(selection) == 'O') {
      input1.data = 0.04;
      input2.data = 0.04;
    }
    else{
      std::cout << "Quitting.." << std::endl;
      ros::shutdown();
    }

    ROS_INFO_STREAM ("Sending command:\n" << input1 << input2);

    gripper1_pub.publish(input1);
    gripper2_pub.publish(input2);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
