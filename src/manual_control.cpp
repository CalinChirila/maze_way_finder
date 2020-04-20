#include "geometry_msgs/Twist.h"
#include "iostream"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "manual_control_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(0.5);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

  std::string userCommand;
  geometry_msgs::Twist command;

  while (ros::ok()) {
    std::cin >> userCommand;
    if (userCommand == "w") {
      command.linear.x = 1.0;
      command.angular.z = 0.0;
    } else if (userCommand == "s") {
      command.linear.x = -1.0;
      command.angular.z = 0.0;
    } else if (userCommand == "a") {
      command.linear.x = 0.0;
      command.angular.z = 1.0;
    } else if (userCommand == "d") {
      command.linear.x = 0.0;
      command.angular.z = -1.0;
    }
    pub.publish(command);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}