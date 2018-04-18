#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>


// Include any additional header or service files

using std::cout;
using std::string;

// Declare class variables and publishers
std_msgs::String string_to_publish_;

ros::Publisher string_publisher_;

// Define service and callback functions 

int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment1");
  ros::NodeHandle n;

  // Perform operations defined in Assignment 1

  ros::Rate loop(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
