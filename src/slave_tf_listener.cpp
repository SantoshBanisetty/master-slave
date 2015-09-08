#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"



int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;


  ros::Publisher slave_vel =
    node.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);

  tf::TransformListener listener;

  ros::Rate rate(1000.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/robot_1", "/robot_0",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                   transform.getOrigin().x());
    vel_msg.linear.x = 1.0 * sqrt(pow(transform.getOrigin().x(), 2) +
                                  pow(transform.getOrigin().y(), 2));
    slave_vel.publish(vel_msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};



