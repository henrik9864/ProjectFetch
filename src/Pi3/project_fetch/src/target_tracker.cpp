#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

struct Navigation {
  float turnAngles;
  ros::Publisher pubControl;

  Navigation(ros::NodeHandle nh) {
    turnAngles = 0;
    pubControl = nh.advertise<geometry_msgs::Twist>("car_ctrl", 2);
  }

  void control() {
    geometry_msgs::Twist output;
    if (turnAngles != 0) {
      
    }
    pubControl.publish(output);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_tracker");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  Navigation navigator;
  while(ros::ok()) {
    navigator.control();
    ros::spinOnce();
  }
  return 0;
}
