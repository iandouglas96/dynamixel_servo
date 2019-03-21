#include <ros/ros.h>
#include "dynamixel_servo/dynamixel_servo.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "dynamixel_servo");
  ros::NodeHandle nh("~");

  try {
    DynamixelServo node(nh);
    node.initialize();
    ros::spin();
  } catch (const std::exception& e) {
    ROS_ERROR("%s: %s", nh.getNamespace().c_str(), e.what());
  }
  return 0;
}
