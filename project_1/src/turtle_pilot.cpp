#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <stdio.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <kobuki_msgs/BumperEvent.h>

#include "turtle_sense.h"
#include "turtle_behaviors.h"

static const double LINEAR_SPEED  = 0.3;
static const double ANGULAR_SPEED = 0.3;

static const double OBSTACLE_DIST = 2.0;

// Behavior prioritization
int selectBehavior(TurtleSense& sense) {
    // Check for emergency halt
    if (sense.hasHitObstacle()) {
        sense.resetHitFlag();   // Acknowledge bumper hit
        return 0;
    }
    // Check for obstacle nearby
    else if (sense.distToObstacleFeet() < OBSTACLE_DIST) {
        // Approaching symmetric obstacle
        if (sense.isSymmetricApproach())
            return 1;
        // Approaching asymmetric obstacle
        else
            return 2;
    }
    // Drive forward
    else
        return 3;
}


int main(int argc, char** argv) {
    //initialize the ROS node
    ROS_INFO_STREAM("Turtlebot Pilot");
    ros::init(argc, argv, "turtle_pilot");
    ros::NodeHandle nh;

    TurtleSense sense(nh);
    TurtleBehaviors behave(nh, LINEAR_SPEED, ANGULAR_SPEED);
    ros::Rate rate(10); // in Hz
    
    while(ros::ok()) {
        switch (selectBehavior(sense)) {
            case 0:     behave.halt();  sense.resetHitFlag();           break;
            case 1:     behave.escape();                                break;
            case 2:     behave.avoid(sense.nearestObstacleLocation());  break;
            default:    behave.drive();                                 break;
        }

        behave.clock();
        ros::spinOnce();
        rate.sleep();
    }
}