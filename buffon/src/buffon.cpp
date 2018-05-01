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
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

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

float PI = 3.1415926535897;

float ballXVelocity = 0.0; //Velocity of the ball
float timeBetween = 0.1; //fix this tentaive / unsure

bool somethingMovedCloser = false; //used to indicate when a new obect is closer to the robot

sensor_msgs::PointCloud baseScan;// builds up a basescan for the robot to compare with  
sensor_msgs::PointCloud threshold;

geometry_msgs::Point32 point1;
geometry_msgs::Point32 point2; //measures the difference between these two to find the balls speed once the new scan has become different than the basescan

std::vector<sensor_msgs::PointCloud> clouds;
Vector3f motionVector; //Vector of motion of the ball

geometry_msgs::Twist twist;

float numberOfBallScans = 0;

// For Interception
float interception_time; //Time to intercept the ball
float ballX; //Start x position of ball
float ballY; //Start y position of ball
float robotSpeed; //Speed of robot
float ballSpeed; //Speed of ball
Vector3f interceptionPoint; // Point of interception of ball
Vector3f initialPossition;
float angle;
int numberOfPoints = 0;
ros::Publisher pub;
ros::Publisher twistPublisher;

void calculateSpeed()
{
	motionVector = Vector3f(point2.x - point1.x,
	 point2.y - point2.y, 
	 point2.z - point1.z);

	ballXVelocity = motionVector.z() / timeBetween; //TODO calculate time
}

std::vector<geometry_msgs::Point32> findBall(std::vector<geometry_msgs::Point32> points)
{
	std::vector< vector<geometry_msgs::Point32> > likeZ;
	std::vector<geometry_msgs::Point32> uniqueZ;
	std::vector<geometry_msgs::Point32> ball;
	bool sameZ = false;

	if(points.size() == 0)
	{
		geometry_msgs::Point32 point;
		point.x = 0;
		point.y =0;
		point.z =0; 

		points.push_back(point);
	}
	uniqueZ.push_back(points[0]);
	likeZ.push_back(uniqueZ);

	for(int i = 0; i < points.size(); i++)
	{
		for(int j = 0; j < likeZ.size(); j++)
		{
			if(likeZ[j][0].z == points[i].z)
			{
				sameZ = true;
			}
			if(sameZ)
			{
		 		std::vector<geometry_msgs::Point32> newUniqueZ = likeZ[j];
		 		newUniqueZ.push_back(points[i]);
		 		likeZ[j] = newUniqueZ;
		 		sameZ = false;
			}
		}
		
	}

	bool allBelow = false;
	
	
	if(likeZ.size() > 0)
	{
		float closestZ = likeZ[0][0].z;
		ball = likeZ[0];

		for(int i = 0; i < likeZ.size(); i++)
		{
			for (int j = 0; j < likeZ[i].size(); j++)
			{
				if(likeZ[i][j].z < closestZ)
				{
					ROS_INFO("updating");
					closestZ = likeZ[i][j].z;
					ball = likeZ[i];
					break;
				}
			}
		}

		return ball;
	}
	return points;
}

sensor_msgs::PointCloud toPointcloud(sensor_msgs::PointCloud2 input)
{

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    sensor_msgs::PointCloud cloud;
    cloud.points.resize(temp_cloud->points.size());

    std::vector<geometry_msgs::Point32> newPoints;

    for(int i = 0; i < temp_cloud->points.size(); i++)
    {
    	geometry_msgs::Point32 point;
    	point.x = temp_cloud->points[i].x;
 		point.y = temp_cloud->points[i].y;
 		point.z = temp_cloud->points[i].z;

 		if(point.x < 1 && point.x > -1)
 		{	
 			if(point.y > 0 && point.y < .24) // set to ground height
 			{
 				if(point.z <5)
 				{
 					newPoints.push_back(point);
 				}
 			}
 		}
 	}

 	
 	newPoints = findBall(newPoints);
 	numberOfPoints = newPoints.size();
 	cloud.points = newPoints;
 	return cloud;
 }


void findZPoint1(sensor_msgs::PointCloud cloud)
{
	point1 = cloud.points[0];
	initialPossition = Vector3f(point1.x,point1.y,point1.z);
}

void findZPoint2(sensor_msgs::PointCloud cloud)
{
	point2 = cloud.points[0];
}

void turn()
{
	angle = atan2((interceptionPoint.x() - 0.0), (interceptionPoint.y() - 0.0));
	float speed = .5; // set the speed equal to max rotational velocity
 	float angular_speed = 2;
	float relative_angle = angle;
	
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
	float distance = sqrt((interceptionPoint.x() - 0)*(interceptionPoint.x() - 0) + (interceptionPoint.y() -0)*(interceptionPoint.y() - 0));
	
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
		ros::Duration diff = t1-t0;
		currentDistanceTraveled = .5 * (diff.toSec());
	}
	
	twist.linear.x = 0;
	twistPublisher.publish(twist);
}

void moveToIntecept()
{
	turn();
	goStraight();
}

float distance(float x1, float y1, float x2, float y2)
{
	return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
}

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

void evaluateScan(sensor_msgs::PointCloud2 pcl2)
{ 
	sensor_msgs::PointCloud cloud;

	if(numberOfBallScans < 2)
	{
		if(numberOfPoints < 10)
		{
			sensor_msgs::PointCloud cloud;
			cloud = toPointcloud(pcl2);
			std::vector<geometry_msgs::Point32> points = cloud.points;
			std::vector<geometry_msgs::Point32> newPoints;
			cloud.header = pcl2.header;
		
			pub.publish(cloud);
		}

		numberOfPoints = 0;
		numberOfBallScans++;

		clouds.push_back(cloud);
	}

	findZPoint1(clouds[1]);
	findZPoint2(clouds[2]);

	Vector3f robot = Vector3f(0,0,0);
	calculateSpeed();
	interceptBall(robot,initialPossition, motionVector, ballSpeed, .5); //fix this
	moveToIntecept();
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "BuffonBot");
  ros::NodeHandle n;

  twistPublisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
  pub =n.advertise<PointCloud>("/cloud", 1);

  ros::Subscriber sub = n.subscribe("/camera/depth/points", 1000, evaluateScan);
  	
  
  // Vector3f ball = Vector3f(4,1,0);
  // Vector3f heading = Vector3f(6,7,0);


  //interceptBall(robot, ball, heading, 0, 1.1);
   	
  ros::spin();

  return 0;
}
