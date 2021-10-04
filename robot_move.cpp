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

double min = 0; //minimum value of LaserScan data
double max = 0; //maximum value of LaserScan data
double mid = 0; // medium value of LaserScan data

ros::Subscriber laser_subscriber; //subscriber to laserScan topic
ros::Subscriber sub_odometry; //subscriber to Odometry topic
geometry_msgs::Pose2D current_pose; //current position object of the turtlebot
double first_angle = 0; //original angle of where the turltebot is
double goal_angle = 0; //the desired angle the turtlebot needs to turn.
bool check = true; 
bool hitWall = true;

//continuosly called to update LaserScan data
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  min = scan->ranges[0];
  max = scan->ranges[scan->ranges.size()-1];
  mid = ceil(scan->ranges[(scan->ranges.size()-1)/2]);
  ROS_INFO_STREAM("Min: " << min);
  ROS_INFO_STREAM("Mid: " << mid);
  ROS_INFO_STREAM("Max: " << max);
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

    ROS_INFO_STREAM("Current Position X is: " << current_pose.x);

    ROS_INFO_STREAM("Current Position Y is: " << current_pose.y);    

   
  // angular position
  current_pose.theta = tf::getYaw(msg->pose.pose.orientation);
  ROS_INFO_STREAM("Current Position Theta is: " << current_pose.theta);
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
  ROS_INFO_STREAM("min when it starts:" <<min);
  ROS_INFO_STREAM("max when it starts:" <<max);
  ROS_INFO_STREAM("midwhen it starts:" <<mid);

  ROS_INFO_STREAM("Here before I enter the while loop");
  while(ros::ok()) { 

    // When the turtlebot hits the wall, its linear and angular velocity will be set to 0 (stop moving)
    if(hitWall == true)
    {
      
      base_cmd.linear.x = 0;
      base_cmd.angular.z = 0;
      cmd_vel_pub_.publish(base_cmd);
      hitWall = false;
      ROS_INFO_STREAM("I hit the wall, and I stopped!");
    }
    // When the turtlebot gets close to 1 foot of the wall, it should turn and rotate towards the side with larger LaserScan value 
    else if(min < 0.5)
    {
      
      time_t start_time;
      time_t end_time;

      time(&start_time);
      end_time = start_time + 6;
      while(start_time < end_time)
      {
        base_cmd.linear.x = 0;
        base_cmd.angular.z = 0.4;
        cmd_vel_pub_.publish(base_cmd);
        ROS_INFO_STREAM("ANGULAR1: " << base_cmd.angular.z);
        time(&start_time);
      }
    }

    else if(mid < 0.5)    {
      time_t start_time;
      time_t end_time;

      time(&start_time);
      end_time = start_time + 6;

      while(start_time < end_time)
      {
        base_cmd.linear.x = 0;
        base_cmd.angular.z = 0.5;
        cmd_vel_pub_.publish(base_cmd);
        ROS_INFO_STREAM("ANGULAR2: " << base_cmd.angular.z);
        time(&start_time);
      }
    }

    else if( max < 0.5) //asymetric
    {
      
      time_t start_time;
      time_t end_time;
    
      time(&start_time);
      end_time=start_time+6;
      while(start_time < end_time)
      {
        base_cmd.linear.x = 0;
        base_cmd.angular.z = -0.4;
        cmd_vel_pub_.publish(base_cmd);
        ROS_INFO_STREAM("ANGULAR3: " << base_cmd.angular.z);
        time(&start_time);
      }
    }
    
    else if(min < 0.5 && mid < 0.5 && max < 0.5)
    {
      time_t start_time;
      time_t end_time;
    
      time(&start_time);
      end_time=start_time+6;
      while(start_time < end_time)
      {
        base_cmd.linear.x = 0;
        base_cmd.angular.z = -0.4;
        cmd_vel_pub_.publish(base_cmd);
        ROS_INFO_STREAM("ANGULAR4: " << base_cmd.angular.z);
        time(&start_time);
      }
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