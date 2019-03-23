#include "dynamixel_servo/dynamixel_servo.h"

DynamixelServo::DynamixelServo(ros::NodeHandle &nh) {
  nh_ = nh;
}

void DynamixelServo::regularFreqCallback(const ros::TimerEvent&) {
  //ROS_INFO_STREAM("pos: " << servo->getPosition());
  sensor_msgs::JointState msg;
  msg.header.seq = angle_pub_seq_++;
  msg.header.stamp = ros::Time::now();
  int pos = servo->getPosition();
  if (pos < 0) {
    ROS_WARN("Bad Packet from Servo");
  }
  msg.position.push_back(pos);
  angle_pub_.publish(msg);

  //Sweep
  if (sweeping_) {
    if (pos < lower_stop_ + 10) {
      servo->setPosition(upper_stop_, speed_);
    } else if (pos > upper_stop_ - 10) {
      servo->setPosition(lower_stop_, speed_);
    }
  }
}

void DynamixelServo::reconfigureCallback(dynamixel_servo::DynamixelServoConfig &config, uint32_t level) {
  speed_ = config.speed;
  upper_stop_ = config.upper_stop;
  lower_stop_ = config.lower_stop; 
  if (sweeping_)  
    servo->setPosition(lower_stop_, speed_);
}

void DynamixelServo::initialize() {
  nh_.param<int>("baud_rate", baud_, B1000000);
  nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
  angle_sub_ = nh_.subscribe("angle", 5, &DynamixelServo::angleCallback, this);
  sweep_mode_sub_ = nh_.subscribe("sweep_mode", 5, &DynamixelServo::sweepModeCallback, this);
  angle_pub_ = nh_.advertise<sensor_msgs::JointState>("current_angle", 10);
  pos_timer_ = nh_.createTimer(ros::Duration(0.01), &DynamixelServo::regularFreqCallback, this);

  angle_pub_seq_ = 0;
  sweeping_ = true;
  
  servo = new Servo(port_.c_str(), baud_);

  //Dynamic reconfigure stuff
  auto f = boost::bind(&DynamixelServo::reconfigureCallback, this, _1, _2);
  server_.setCallback(f);

  ROS_INFO("Servo Up");
}

void DynamixelServo::sweepModeCallback(const std_msgs::Bool::ConstPtr& msg) {
  sweeping_ = msg->data;
  if (sweeping_) 
    servo->setPosition(lower_stop_, speed_);
}

void DynamixelServo::angleCallback(const std_msgs::Int16::ConstPtr& msg) {
  ROS_INFO("Servo Manual Override");
  sweeping_ = false;
  servo->setPosition(msg->data, 512);
}
