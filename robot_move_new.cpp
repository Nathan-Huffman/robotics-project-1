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
#include <kobuki_msgs/BumperEvent.h>

double min; //minimum value of LaserScan data
double max; //maximum value of LaserScan data
double mid; // medium value of LaserScan data

ros::Subscriber laser_subscriber; //subscriber to laserScan topic
ros::Subscriber sub_odometry; //subscriber to Odometry topic
geometry_msgs::Pose2D current_pose; //current position object of the turtlebot
double first_angle; //original angle of where the turltebot is
double goal_angle = 0; //the desired angle the turtlebot needs to turn.
bool check = true;  
bool hitWall;


//continuosly called to update LaserScan data
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  min = scan->ranges[0];
  max = scan->ranges[scan->ranges.size()-1];
  mid= ceil(scan->ranges[(scan->ranges.size()-1)/2]);
  ROS_INFO_STREAM("Min: " <<min);
  ROS_INFO_STREAM("Mid: " <<mid);
  ROS_INFO_STREAM("Max: " <<max);

}
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& bump)
{
  if(bump->state==1)
  {
      hitWall=true;
  }
  else{
    hitWall=false;
  }
  ROS_INFO_STREAM(hitWall);

}

//continuosly called to update Odometry data
void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
   
  // angular position
  current_pose.theta = tf::getYaw(msg->pose.pose.orientation);
  ROS_INFO_STREAM("THETA is: " << current_pose.theta);
}

int main(int argc, char** argv)
{
  //initialize the ROS node
  ROS_INFO_STREAM("Turtlebot Mover");
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  //initialize publisher
  ros::Publisher cmd_vel_pub_= nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
  
    
  geometry_msgs::Twist base_cmd;
  
  laser_subscriber = nh.subscribe("/scan", 1000, scanCallback);
  sub_odometry = nh.subscribe("odom", 1, odomCallback);
  ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 1000, bumperCallback);

  ros::Rate rate(10); // 5Hz


  ROS_INFO_STREAM("Here before I enter the while loop");

  while(ros::ok()) { 

    if(hitWall)
    {
      
      base_cmd.linear.x = 0;
      base_cmd.angular.z = 0;
      cmd_vel_pub_.publish(base_cmd);
      hitWall = false;
      ROS_INFO_STREAM("I hit the wall");
    }

    else if((min <= 0.5|| mid<= 0.5|| max <= 0.5) /*&& (min<max)*/)  //asymetric
    {
      if(check)  //check is true
      {
        first_angle = current_pose.theta; // current position angle of where the turtlebot is.
        goal_angle = first_angle + 3.14;
        check = false;
        ROS_INFO_STREAM("Goal theta: " << goal_angle);
      }
      if  (abs(goal_angle - current_pose.theta)>1)
      {
    
        base_cmd.linear.x = 0;
        base_cmd.angular.z = -0.4;
        cmd_vel_pub_.publish(base_cmd);
        ROS_INFO_STREAM("Angular: " << base_cmd.angular.z);
      }
      check=true;
      ROS_INFO_STREAM("min < max");
    }
    else if((min <= 0.5|| mid <= 0.5 || max <= 0.5) && (min>=max))  //asymetric
    {
      if(check)  //check is true
      {
        first_angle = current_pose.theta; // current position angle of where the turtlebot is.
        goal_angle=first_angle + 3.14;
        check=false;
        ROS_INFO_STREAM("Goal theta: " << goal_angle);
      }
      if (abs(goal_angle-current_pose.theta)>1)
      {
        base_cmd.linear.x = 0;
        base_cmd.angular.z = 0.4;
        cmd_vel_pub_.publish(base_cmd);
        ROS_INFO_STREAM("Angular: " << base_cmd.angular.z);
      }
      check=true;

      ROS_INFO_STREAM("min > max");

    }

  else  {
      ROS_INFO_STREAM("speed is " << base_cmd.linear.x);

      base_cmd.linear.x = 0.3;
      base_cmd.angular.z = 0;
      cmd_vel_pub_.publish(base_cmd);

    }

    ros::spinOnce();
    rate.sleep();
    
  }
  
  return 0;
}