#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h" 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <stdio.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

double min;
double max;
double mid;
ros::Subscriber laser_subscriber;
geometry_msgs::Pose2D current_pose;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  min=scan->ranges[0];
  max=scan->ranges[scan->ranges.size()-1];
  mid= ceil(scan->ranges[(scan->ranges.size()-1)/2]);
}
void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    
   
    
    // angular position
    current_pose.theta = tf::getYaw(msg->pose.pose.orientation);
    
}
int main(int argc, char** argv)
{
  //initialize the ROS node
  ROS_INFO_STREAM("Turtlebot Mover");
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  //initialize publisher
  ros::Publisher cmd_vel_pub_= nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    
  //initial direction that turtlebot should go
  geometry_msgs::Twist base_cmd;
  //geometry_msgs::Twist base_cmd_turn_
  //base_cmd.linear.x = 0;
  //base_cmd.angular.z = 0;
  laser_subscriber = nh.subscribe("/scan", 1000, scanCallback);
  ros::Subscriber sub_odometry = nh.subscribe("odom", 1, odomCallback);

  ROS_INFO_STREAM("And Crashing ... ctrl + c to stop me :)");

  
  ; //45 deg/s * 2 sec = 90 degrees 

  ros::Rate rate(10); // 5Hz

  while(ros::ok()) { 

    base_cmd.linear.x = 0.5;
    base_cmd.angular.z = 0;

    if((min<= 0.3048|| mid<= 0.3048 || max<= 0.3048) && (min<max))
    {
    base_cmd.linear.x = 0;
    base_cmd.angular.z = -0.3;
    cmd_vel_pub_.publish(base_cmd);
    }

    

    

    cmd_vel_pub_.publish(base_cmd);
    rate.sleep();
    ros::spinOnce();

  }


  

  return 0;
}