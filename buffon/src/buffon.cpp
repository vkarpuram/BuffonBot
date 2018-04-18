#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Point;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using visualization_msgs::Marker;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud;
using sensor_msgs::PointCloud2;
using std::cout;
using std::vector;
using namespace std;

// Global Parameters
const float gRobotRadius = 0.18;
const float gRobotHeight = 0.36;
const float gEpsilon = 0.15;
const float gAMaxLin = 1 * 0.5; // m/s^2
const float gAMaxRot = 1 * 2.0; // rad/s^2
const float gVMaxLin = 0.5; // m/s
const float gVMaxRot = 1.5; // rad/s
const float gDT = 0.02; // s

sensor_msgs::PointCloud2 scan; 
sensor_msgs::PointCloud2 differences;

void evaluateScan(const sensor_msgs::PointCloud2 cloud)
{ 
  //scan.header = cloud.header;
  //std::vector<geometry_msgs::Point32> points = cloud.points; 
  
  // for(int i = 0; i < points.size();i++){
  // 	ROS_INFO("%d",points[i].x);
  // }
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "BuffonBot"); // topic /camera/depth/points 
  ros::NodeHandle n;
  ros::Rate loop(10); // 10 Hertz

  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, evaluateScan);

  while (ros::ok()) 
  {
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
