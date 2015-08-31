/*
 * slave.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: santosh
 */

//Slave robot will subscribe to master robot's position topic and will 
//implement go to goal behaviour  

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <sstream>

#define PI 3.14
/*
 *Global variables decleartions
 */
double mpX;
double mpY;
double moZ;

double spX;
double spY;
double soZ;

double distance, angle;

double lrange=0,crange=0,rrange=0;
double lintensity=0, cintensity=0, rintensity=0;

ros::Publisher velocity_publisher;
//Function declerations of move and rotate
void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double relative_angle, bool clockwise);
//Function declearations for equilidian distance and degrees to radians conversion
double getDistance(double x1, double y1, double x2, double y2);
double degrees2radians(double angle_in_degrees);


void masterPoseCallBack(const nav_msgs::Odometry::ConstPtr & master_pose);
void slavePoseCallBack(const nav_msgs::Odometry::ConstPtr & slave_pose);
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg);

double desireOrientation (double xM, double yM, double xS, double yS);
void goToGoal(void);

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "slave");

  ros::NodeHandle slaveNode;

  velocity_publisher = slaveNode.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1);
  //ros::Subscriber slave_sub = slaveNode.subscribe("myPose", 1000, goToGoal);
  ros::Subscriber master_pose = slaveNode.subscribe("/robot_0/base_pose_ground_truth", 1, masterPoseCallBack);
  ros::Subscriber slave_pose = slaveNode.subscribe("/robot_1/base_pose_ground_truth", 1, slavePoseCallBack);
  ros::Subscriber laser = slaveNode.subscribe("/robot_1/base_scan", 1, laserCallBack);
  //just for testing

ros::Rate loop_rate(1000);
  while (ros::ok())
  {
  	//move (1.0, 1.0, 1);
  	
	goToGoal();
	ros::spinOnce();
	loop_rate.sleep();
  }
 
  ros::spin();

  return 0;
}

/**
 *  makes the robot turn with a certain linear velocity, for 
 *  a certain distance either forward or backward  
 */
void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
   //set a random linear velocity in the x-axis
   if (isForward)
	   vel_msg.linear.x =abs(speed);
   else
	   vel_msg.linear.x =-abs(speed);
   vel_msg.linear.y =0;
   vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
   vel_msg.angular.x = 0;
   vel_msg.angular.y = 0;
   vel_msg.angular.z =0;

   double t0 = ros::Time::now().toSec();
   double current_distance = 0.0;
   ros::Rate loop_rate(10000);
   do{
	   velocity_publisher.publish(vel_msg);
	   double t1 = ros::Time::now().toSec();
	   current_distance = speed * (t1-t0);
	   ros::spinOnce();
	   loop_rate.sleep();
   }while(current_distance<distance);
   vel_msg.linear.x =0;
   velocity_publisher.publish(vel_msg);

}

/**
 *  makes the robot turn with a certain angular velocity, for 
 *  a certain distance in either clockwise or counter-clockwise direction  
 */
void rotate (double angular_speed, double relative_angle, bool clockwise){
//angular_speed = degrees2radians(angular_speed);
relative_angle = degrees2radians(relative_angle);
	geometry_msgs::Twist vel_msg;
	   //set a random linear velocity in the x-axis
	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;
	   //set a random angular velocity in the y-axis
	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;
	   if (clockwise)
	   		   vel_msg.angular.z =-abs(angular_speed);
	   	   else
	   		   vel_msg.angular.z =abs(angular_speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(10000);
	   do{
		   velocity_publisher.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
		   //cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   velocity_publisher.publish(vel_msg);
}

/**
 *  converts angles from degree to radians  
 */

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

/*
 * get the euclidian distance between two points 
 */
double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

/*
 * Function to calculate desired orientation to reach the master  
 */
double desireOrientation (double xM, double yM, double xS, double yS)
{
return atan2((yM-yS),(xM-xS));
}

/*
 * Call back implementation to read and process master robot's position  
 */
void masterPoseCallBack(const nav_msgs::Odometry::ConstPtr & master_pose)
{
ROS_INFO("I am in: [%s]", "master call back");
mpX = master_pose->pose.pose.position.x;
mpY = master_pose->pose.pose.position.y;
moZ = master_pose->pose.pose.orientation.z;
ROS_INFO("mpX: [%f]", mpX);
ROS_INFO("mpY: [%f]", mpY);
ROS_INFO("moZ: [%f]", moZ);
}

/*
 * Call back implementation to read and process slave robot's position  
 */
void slavePoseCallBack(const nav_msgs::Odometry::ConstPtr & slave_pose)
{
ROS_INFO("I am in: [%s]", "slave call back");
spX = slave_pose->pose.pose.position.x;
spY = slave_pose->pose.pose.position.y;
soZ = slave_pose->pose.pose.orientation.z;
ROS_INFO("spX: [%f]", spX);
ROS_INFO("spY: [%f]", spY);
ROS_INFO("soZ: [%f]", soZ);
}

/*
 * Call back implementation to read and process laser data  
 */
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg)
{
ROS_INFO("I am in: [%s]", "laser call back");
lrange = laser_msg->ranges[2];
crange = laser_msg->ranges[1];
rrange = laser_msg->ranges[0];
ROS_INFO("Ranges: left->[%f], center->[%f], right[%f]", lrange, crange, rrange);
lintensity = laser_msg->intensities[2];
cintensity = laser_msg->intensities[1];
rintensity = laser_msg->intensities[0];
ROS_INFO("Intensities: left->[%f], center->[%f], right[%f]", lintensity, cintensity, rintensity);
}

/*
 * Go to goal implementaion. may be a proportional controller here
 */
void goToGoal(void)
{
	int Kv=1, Kw=2;
	double relative_theta;
	angle = desireOrientation (mpX,mpY,spX,spY);
	ROS_INFO("Orientation: [%f]", angle);
	//rotate (5.0, angle-soZ, 1);
	distance = getDistance(mpX,mpY,spX,spY);
	ROS_INFO("Distance: [%f]", distance);
	relative_theta = angle-soZ;
	double v = Kv*distance;
	double w = Kw*relative_theta;
	bool dir = (relative_theta<0)?true:false;
	ROS_INFO("%f,%f", v,w);
	rotate (w, angle-soZ, dir);
	move (v, distance, 1);
  ROS_INFO("I am in: [%s]", "goal pursuit mode");
}
