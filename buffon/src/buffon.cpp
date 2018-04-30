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
//#include <pcl_ros/point_cloud.h>

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
float buildupScanLimit = 3; //10 ?
float reset = 0;
float resetLimit = 5;

float PI = 3.1415926535897;

float ballXVelocity = 0.0; //Velocity of the ball
float timeBetween = 0.1; //fix this tentaive / unsure

bool somethingMovedCloser = false; //used to indicate when a new obect is closer to the robot

sensor_msgs::PointCloud baseScan;// builds up a basescan for the robot to compare with  
sensor_msgs::PointCloud threshold;

geometry_msgs::Point32 closestPoint;
geometry_msgs::Point32 closestPoint2; //measures the difference between these two to find the balls speed once the new scan has become different than the basescan

Vector3f motionVector; //Vector of motion of the ball

geometry_msgs::Twist twist;

// For Interception
float interception_time; //Time to intercept the ball
float ballX; //Start x position of ball
float ballY; //Start y position of ball
float robotSpeed; //Speed of robot
float ballSpeed; //Speed of ball
Vector3f interceptionPoint; // Point of interception of ball

float angle;

ros::Publisher pub;
ros::Publisher twistPublisher;

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
	image.header = cloud.header;
	pub.publish(cloud);

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

void turn()
{
	angle = atan2((interceptionPoint.x() - 0.0), (interceptionPoint.y() - 0.0));
	float speed = 2; // set the speed equal to max rotational velocity
 	
 	float angular_speed = speed*2*PI/360;
	float relative_angle = angle * 2 * PI / 360;
	
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;

	twist.angular.x = 0;
	twist.angular.y = 0;

	if(angle < 90) // maybe? this determines which way it turns 
	{
       twist.angular.z = -abs(angular_speed);
    }
    else
    {
       twist.angular.z = abs(angular_speed);
    }
   
	ros::Time t0 = ros::Time::now();
    float current_angle = 0;

    while(current_angle < relative_angle)
    {

      	twistPublisher.publish(twist);
        ros::Time t1 = ros::Time::now();
		ros::Duration diff=t1-t0;
        current_angle = angular_speed * (diff.toSec());
    }


      twist.angular.z = 0;
      twistPublisher.publish(twist);
}

void goStraight()
{
	float distance = sqrt((interceptionPoint.z() - 0)*(interceptionPoint.z() - 0) + (interceptionPoint.x() -0)*(interceptionPoint.x() - 0));
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	twist.linear.y = 0;
	twist.linear.z = 0;


	float currentDistanceTraveled = 0;
 	ros::Time t0 = ros::Time::now();
	twist.linear.x = 0.5;

	
	while(currentDistanceTraveled < distance)
	{
		
		twistPublisher.publish(twist);
		ros::Time t1 = ros::Time::now();
		ros::Duration diff=t1-t0;
		currentDistanceTraveled = currentDistanceTraveled + .5 * (diff.toSec());
	}

	twist.linear.x = 0;
	twistPublisher.publish(twist);
}

void moveToIntecept()
{

	turn();
	goStraight();
	ROS_INFO("moved");
}



// float distance(Vector3f& one, Vector3f& two){

// 	return sqrt(pow(two.x()-one.x(),2) + pow(two.y()-one.y(),2));
// }

float distance(float x1, float y1, float x2, float y2){
	return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
}



// Interception of ball
void interceptBall(Vector3f& robot, Vector3f& ball, Vector3f& heading, float ballVelocity, float robotVelocity){

	float sin_Ball = (((robot.x()-ball.x()) * (heading.y()-ball.y())) - ((robot.y()-ball.y())*(heading.x()-ball.x())))/((robot-ball).norm() * (heading-ball).norm());

	float sin_Robot = (ballVelocity / robotVelocity) * sin_Ball;
	
	// If the ball is moving too fast to intercept, print an error message and return
	if(abs(sin_Robot) > 1){
		ROS_INFO("Cannot Intercept Ball, too fast");
		return;
	}

	// Calculate the point of interception
	else{

		float sin_interceptPoint = (sin_Robot * sqrt(1-pow(sin_Ball,2)) + sin_Ball * sqrt(1-pow(sin_Robot, 2)));
		

		float ballToDest = distance(ball.x(),ball.y(), robot.x(),robot.y()) * (sin_Robot / sin_interceptPoint); // The distance from the ball to the destination


		// If the ball reaches the destination before it can be caught, print an error and return
		if(ballToDest > distance(robot.x(),robot.y(),heading.x(),heading.y())){
			ROS_INFO("Cannot Intercept Ball. Ball reaches destination before being caught by robot");
			return;
		}
		//Calculate the point of interception
		else{
			interception_time = ballToDest/ballVelocity; // Gives the time of interception of the ball
			interceptionPoint = ball + ballToDest *(heading-ball)/(heading - ball).norm(); // gives the point of interception of the ball
		}
	}
	//ROS_INFO("Intercepted ball");
	return;
}



int main(int argc, char **argv) 
{
  ros::init(argc, argv, "BuffonBot"); // topic /camera/depth/points 
  ros::NodeHandle n;
 
  
  // cout << "Type \"goalie\" for Goalie Mode!\n or \nType\"avoid\" for Evasion Mode!\n\n";
  // string input;
  // cin >> input;


  // if("goalie" == input)
  // {
  	twistPublisher = n.advertise<geometry_msgs::Twist>("/cmg_Imaged_vel_mux/input/navi", 1000);
  	pub =n.advertise<PointCloud>("/cloud", 1);

  	ros::Subscriber sub = n.subscribe("/camera/depth/image", 1000, evaluateScan);
  	

  	Vector3f robot = Vector3f(0,0,0);
  	Vector3f ball = Vector3f(4,1,0);
  	Vector3f heading = Vector3f(6,7,0);


  	interceptBall(robot, ball, heading, 0, 1.1);
  	moveToIntecept();
  	ROS_INFO("%f %f", interceptionPoint.x(), interceptionPoint.y());

	ros::spin();
  //}
  // if("avoid" == input)
  // {

  // }





  return 0;
}
