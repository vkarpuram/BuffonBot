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
#include <sensor_msgs/Image.h>
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
using sensor_msgs::Image;
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

//camera paramaters
float value1 = 3.008;
float value2 = -0.002745;
float px = 320.0;
float py = 240.0;
float fx = 588.446;
float fy = -564.227;

float buildup = 0;
float buildupScanLimit = 3; //10 ?
//fix this tentaive / unsure
float timeBetween = 0.1;

//sensor_msgs::point_cloud_conversion PCC;
sensor_msgs::PointCloud baseScan; 
sensor_msgs::PointCloud threshold;

bool somethingMovedCloser = false;

geometry_msgs::Point32 closestPoint;
geometry_msgs::Point32 closestPoint2;

float ballXVelocity = 0.0;

Vector3f motionVector;

void closestPointInBase()
{
	closestPoint = baseScan.points[0];
	for(int i = 1; i < baseScan.points.size(); i++)
	{
		if(baseScan.points[i].x < closestPoint.x)
		{
			closestPoint = baseScan.points[i];
		}
	}	
	ROS_INFO("x = %f", closestPoint.x);
}

void calculateSpeed()
{
	motionVector = Vector3f(closestPoint.x - closestPoint2.x,
	 closestPoint.y - closestPoint2.y, 
	 closestPoint.z - closestPoint2.z);

	ballXVelocity = motionVector.x() / timeBetween;
	
	//TODO: check this ???
}

void findClosestInNewScan()
{
	for(int i = 0; i < threshold.points.size(); i++)
	{
		if(threshold.points[i].x > closestPoint.x)
		{
			somethingMovedCloser = true;
			closestPoint = threshold.points[i];
		}
	}
}

void findCloserPoint(std::vector<geometry_msgs::Point32> points)
{
	somethingMovedCloser = false;
	for(int i = 0; i < points.size(); i++)
	{
		if(points[i].x > closestPoint.x)
		{
			somethingMovedCloser = true;
			closestPoint2 = points[i]; 
		}
	}
	
}

sensor_msgs::PointCloud imageToCloud(sensor_msgs::Image image)
{
  sensor_msgs::PointCloud cloud;
  std::vector<geometry_msgs::Point32> points;

  const int image_width = image.width;
  const int image_height = image.height;
  cloud.points.resize(image_width * image_height);

  int i = 0;
  for (int y = 0; y < image_height; ++y) {
    for (int x = 0; x < image_width; ++x) {
      uint16_t byte0 = image.data[2 * (x + y * image_width) + 0];
      uint16_t byte1 = image.data[2 * (x + y * image_width) + 1];
      if (!image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      
      const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;
      cloud.header.frame_id = "kinect_1";

      geometry_msgs::Point32 point;

      const float Z = 1.0 / (value1 + value2 * static_cast<float>(raw_depth));
      point.z = Z * (static_cast<float>(x) - px) / fx;
      point.y = Z * (static_cast<float>(y) - py) / fy;
      point.x = Z;
    
      cloud.points[i] = point;
      
      i++;
    }
  }
	return cloud;
}

void evaluateScan(sensor_msgs::Image image)
{ 
	ROS_INFO("Scanning");
	sensor_msgs::PointCloud cloud = imageToCloud(image);

	std::vector<geometry_msgs::Point32> points = cloud.points;
	std::vector<geometry_msgs::Point32> newPoints;

	if(buildup < buildupScanLimit) 
	{
		baseScan.points.insert(baseScan.points.end(), cloud.points.begin(), cloud.points.end());

	 	buildup++;
	 	return;
	}

	if(buildup == buildupScanLimit)
	{
		ROS_INFO("Base Scan Complete");
		closestPointInBase();
		buildup++;
	}

	if(somethingMovedCloser)
	{
		ROS_INFO("Something has Moved Closer");
		findCloserPoint(points);
		calculateSpeed();
		
		return;
	}

	for(int i = 0; i < points.size(); i++)
	{
		geometry_msgs::Point32 point;

		point.x = points[i].x;
		point.y = points[i].y;
		point.z = points[i].z;
		
		newPoints.push_back(point);
	}

	threshold.points = newPoints;
	findClosestInNewScan();
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "BuffonBot"); // topic /camera/depth/points 
  ros::NodeHandle n;
 
  ros::Subscriber sub = n.subscribe("/camera/depth/image", 1000, evaluateScan);
 
  ros::spin();

  return 0;
}
