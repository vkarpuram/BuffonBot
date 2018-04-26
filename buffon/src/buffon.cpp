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

// Global Robot Parameters
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

//Determines how many baseScans will be taken
float buildup = 0;
float buildupScanLimit = 10; //10 ?
float reset = 0;
float resetLimit = 5;

float ballXVelocity = 0.0;
float distance2 = 0.0;
float zLoc = 0.0;
float timeBetween = 0.1; //fix this tentaive / unsure

bool somethingMovedCloser = false; //used to indicate when a new obect is closer to the robot

sensor_msgs::PointCloud baseScan;// builds up a basescan for the robot to compare with  
sensor_msgs::PointCloud threshold;

geometry_msgs::Point32 closestPoint;
geometry_msgs::Point32 closestPoint2; //measures the difference between these two to find the balls speed once the new scan has become different than the basescan

Vector3f motionVector;
geometry_msgs::Twist twist;
ros::Publisher pub;

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
	//ROS_INFO("x = %f", closestPoint.x);
}

void publishTwist()
{
	pub.publish(twist);
}

void turn()
{
	ros::Time time;
	geometry_msgs::Vector3 ang;

	geometry_msgs::Vector3 lin;
	lin.x = 0;
	lin.y = 0;
	lin.z = 0;

	float angular_speed = 1*2*3.14/360;
    float relative_angle = 90*2*3.14/360;
	
	ang.x = 0;
	ang.y = 0;
 	ang.z = -abs(angular_speed);
 
     
    float t0 = time.now().toSec();
    float current_angle = 0;
   
    while(current_angle < relative_angle)
    {
        pub.publish(twist);
       	float t1 = time.now().toSec();
      	current_angle = angular_speed*(t1-t0);
	}

	twist.angular.z = 0;
	pub.publish(twist);

	publishTwist();
}

void go()
{
	while(distance2 != 0)
	{
		geometry_msgs::Vector3 ang;
		ang.x = 0;
		ang.y = 0;
		ang.z = 0;

		geometry_msgs::Vector3 lin;
		ang.x = 1;
		ang.y = 0;
		ang.z = 0;

		twist.angular = ang;
		twist.linear = lin;
		distance2 = distance2 - 10;
		publishTwist();
	}
}

void createTwistTest()
{
	turn();
	go();	
}

void calculateSpeed()
{
	motionVector = Vector3f(closestPoint.x - closestPoint2.x,
	 closestPoint.y - closestPoint2.y, 
	 closestPoint.z - closestPoint2.z);

	ballXVelocity = motionVector.x() / timeBetween;
	distance2 = closestPoint2.x;
	zLoc = closestPoint2.z;

	//createTwistTest();
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
	//ROS_INFO("Scanning");
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

		return;
	}

	if(somethingMovedCloser)
	{
		ROS_INFO("Something has Moved Closer");
		findCloserPoint(points);
		calculateSpeed();
		
		reset = 0;
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
	reset = reset + 1;

	if(reset == resetLimit)
	{
		buildup = 0;
	}

	findClosestInNewScan();
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "BuffonBot"); // topic /camera/depth/points 
  ros::NodeHandle n;
 
  
 // cout << "Type \"goalie\" for Goalie Mode!\n or \nType\"avoid\" for Evasion Mode!\n\n";
  //string input;
  //cin >> input;


  // if("goalie" == input)
  // {
  	ros::Subscriber sub = n.subscribe("/camera/depth/image", 1000, evaluateScan);
  	pub = n.advertise<geometry_msgs::Twist>("/cmg_Imaged_vel_mux/input/navi", 1000);

  	ros::spin();

  


  //}
  // if("avoid" == input)
  // {

  // }

  return 0;
}
