//Gripper controller
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <iostream>


int main(int argc, char **argv) {

  ros::init(argc, argv, "gripper");

  ros::NodeHandle n;

  ros::Publisher gripper_pub = n.advertise<trajectory_msgs::JointTrajectory>("/panda/panda_hand_controller/command", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok()){

    trajectory_msgs::JointTrajectory input;

    input.joint_names.clear();
    input.joint_names.push_back("panda_finger_joint1");
    input.joint_names.push_back("panda_finger_joint2");

    input.points.resize(1);
    input.points[0].positions.resize(input.joint_names.size(), 1.0);
    ROS_INFO_STREAM ("Sending command:\n" << input);

    gripper_pub.publish(input);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
