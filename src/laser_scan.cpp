#include "fstream"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <ros/package.h>

float front_sensor_value1, front_sensor_value2, front_sensor_value3,
    right_sensor_value, left_sensor_value, linear_velocity, angular_velocity,
    reverse_velocity, reverse_treshold, smooth_turn_treshold;

geometry_msgs::Twist command;

void setConfigurationValues(const char *filePath) {
  std::string line, key;
  float value;
  std::ifstream filestream(filePath);

  if (filestream.is_open()) {
    while (std::getline(filestream, line)) {
      std::istringstream linestream(line);
      linestream >> key >> value;

      if (key == "linear_velocity") {
        ROS_INFO("Linear velocity: %f", value);
        linear_velocity = value;
      } else if (key == "angular_velocity") {
        angular_velocity = value;
        ROS_INFO("Angular velocity: %f", value);
      } else if (key == "reverse_velocity") {
        ROS_INFO("Reverse velocity: %f", value);
        reverse_velocity = value;
      } else if (key == "reverse_treshold") {
        ROS_INFO("Reverse treshold: %f", value);
        reverse_treshold = value;
      } else if (key == "smooth_turn_treshold") {
        ROS_INFO("Smooth turn treshold: %f", value);
        smooth_turn_treshold = value;
      }
    }
  } else {
    ROS_ERROR("Couldn't read config file!");
  }
}

void get_scanner_values(const sensor_msgs::LaserScan::ConstPtr &scanMsg) {
  // Scan in a cone shape to avoid colliding with a corner
  front_sensor_value1 = scanMsg->ranges[scanMsg->ranges.size() / 2 - 20];
  front_sensor_value2 = scanMsg->ranges[scanMsg->ranges.size() / 2];
  front_sensor_value3 = scanMsg->ranges[scanMsg->ranges.size() / 2 + 20];

  // Values for a 360 laser scanner
  left_sensor_value = scanMsg->ranges[scanMsg->ranges.size() * 3 / 4];
  right_sensor_value = scanMsg->ranges[scanMsg->ranges.size() * 1 / 4];
}

void move() {
  command.linear.x = -1 * linear_velocity;
  if (front_sensor_value1 < 1 || front_sensor_value2 < 1 ||
      front_sensor_value3 < 1) {
    command.linear.x = -1 * linear_velocity / 2;
    // Reverse if front sensor values are lower than 0.3
    if (front_sensor_value1 < reverse_treshold ||
        front_sensor_value2 < reverse_treshold ||
        front_sensor_value3 < reverse_treshold) {
      ROS_INFO("Reversing");
      command.angular.z = 0.0;
      command.linear.x = reverse_velocity;
    }

    // Turn right if more space
    else if (right_sensor_value > left_sensor_value) {
      ROS_INFO("Turning right");
      command.angular.z = -1 * angular_velocity; // turns right
    } else {
      ROS_INFO("Turning left");
      command.angular.z = angular_velocity; // turns left
    }
  } else {
    // Move forward
    command.angular.z = 0.0;

    if (left_sensor_value < smooth_turn_treshold) {
      command.angular.z = -1 * angular_velocity; // Turn right while moving
    }

    if (right_sensor_value < smooth_turn_treshold) {
      command.angular.z = angular_velocity; // Turn left while moving
    }
  }
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scanMsg) {
  ros::NodeHandle nh;
  ros::Publisher move_publisher =
      nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Rate loop_rate(0.5);

  get_scanner_values(scanMsg);
  move();

  // Execute the command
  move_publisher.publish(command);

  ros::spinOnce();
  loop_rate.sleep(); // Enforce loop rate
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_scan_node");
  ros::NodeHandle nh;

  std::string configFilePath =
      ros::package::getPath("maze_way_finder") + "/Config.txt";

  // Read velocity values from config file
  setConfigurationValues(configFilePath.c_str());

  ros::Subscriber sub = nh.subscribe("/scan", 10, scan_callback);

  ros::spin();

  return 0;
}