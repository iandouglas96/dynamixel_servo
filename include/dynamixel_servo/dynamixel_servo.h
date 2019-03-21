#ifndef DYNAMIXEL_SERVO_NODE_H
#define DYNAMIXEL_SERVO_NODE_H

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include "dynamixel_servo/servo.h"
#include <dynamic_reconfigure/server.h>
#include "dynamixel_servo/DynamixelServoConfig.h"

class DynamixelServo {
  public:
    DynamixelServo(ros::NodeHandle &nh);
    void initialize();
  private:
    void angleCallback(const std_msgs::Int16::ConstPtr& msg);
    void sweepModeCallback(const std_msgs::Bool::ConstPtr& msg);
    void regularFreqCallback(const ros::TimerEvent&);
    void reconfigureCallback(dynamixel_servo::DynamixelServoConfig &config, uint32_t level);

    ros::Subscriber angle_sub_, sweep_mode_sub_;
    ros::Publisher angle_pub_;
    ros::Timer pos_timer_;
    dynamic_reconfigure::Server<dynamixel_servo::DynamixelServoConfig> server_;

    int angle_pub_seq_;

    int baud_;
    std::string port_;

    //Dynamic params
    int speed_;
    int upper_stop_;
    int lower_stop_; 

    bool sweeping_;

    Servo *servo;
    ros::NodeHandle nh_;
};

#endif //DYNAMIXEL_SERVO_NODE_H
