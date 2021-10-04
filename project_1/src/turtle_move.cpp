#include "turtle_move.h"

#include <angles/angles.h>

void TurtleMove::MoveListener::odomCallback(const nav_msgs::OdometryConstPtr& msg) {    
    is_valid = true;     // Flag current_pose as having valid data

    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

    // angular position
    current_pose.theta = tf::getYaw(msg->pose.pose.orientation);
}

bool TurtleMove::MoveListener::ensure_valid() {    
    ros::Rate wait(10);     // Used to wait for messages
    for (int timeout = 10; !is_valid && timeout > 0; --timeout) {
        ros::spinOnce();
        wait.sleep();
    }
    return is_valid;        // Flag current_pose data valid
}

// --------------------------------------

TurtleMove::TurtleMove(ros::NodeHandle& nh, double linear_velocity, double angular_velocity) {
    this->linear_velocity_default = linear_velocity;
    this->angular_velocity_default = angular_velocity;

    this->sub_odometry = nh.subscribe("/odom", 30, &MoveListener::odomCallback, &listener);
    this->pub_cmd_vel= nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 10);
}

void TurtleMove::clock() {
    if (is_targetting_angle) {
        current_angle_diff = angles::shortest_angular_distance(listener.current_pose.theta, target_angle);
        if (abs(current_angle_diff) < ANGULAR_TARGETTING_THRESHOLD) {
            turnStop();     // If target angle hit, stop checking and turning
        }
        else {
            double turn_velocity = current_angle_diff * angular_velocity_default * ANGULAR_TARGETTING_GAIN;
            base_cmd.angular.z = std::min(std::max(turn_velocity, -ANGULAR_MAX_VEL), ANGULAR_MAX_VEL); // Clips to max
        }
    }

    traveled_distance = sqrt(pow(checkpoint.x - listener.current_pose.x, 2) + pow(checkpoint.y - listener.current_pose.y, 2));
    
    if (halt_at_target_dist) {
        if (traveled_distance > abs(target_distance)) {
            forwardStop();                  // Stop motion
            halt_at_target_dist = false;    // Remove halt flag if exists
        }
    }
    
    pub_cmd_vel.publish(base_cmd);          // Send comamnds to the robot
}

void TurtleMove::forwardVelocity(double velocity) {
    base_cmd.linear.x = velocity;
}
void TurtleMove::forwardDistVelocityHalt(double distFeet, double velocity, bool halt) {
    halt_at_target_dist = halt;
    if(listener.ensure_valid()) {
        checkpoint = listener.current_pose;
        target_distance = distFeet / 3.2808;    // Feet -> Meters
        base_cmd.linear.x = velocity * ((distFeet > 0) - (distFeet < 0));  // velocity * direction
    }
}
void TurtleMove::forwardStop() {
    base_cmd.linear.x = 0;
}

void TurtleMove::turnVelocity(double velocity) {
    is_targetting_angle = false;
    base_cmd.angular.z = std::min(std::max(velocity, -ANGULAR_MAX_VEL), ANGULAR_MAX_VEL);
}
void TurtleMove::turnDegrees(double turnDegrees) {
    if (listener.ensure_valid()) {
        is_targetting_angle = true;
        target_angle = angles::normalize_angle(listener.current_pose.theta - angles::from_degrees(turnDegrees));
    }
}
void TurtleMove::turnStop() {
    is_targetting_angle = false;
    base_cmd.angular.z = 0;
}

void TurtleMove::stop() {
    forwardStop();
    turnStop();
}

bool TurtleMove::hasTraveledDist() {
    return traveled_distance >= target_distance;
}
bool TurtleMove::hasTargetRotation() {
    return !is_targetting_angle;
}

void TurtleMove::waitTraveledDist() {
    ros::Rate wait(10);
    for (int timeout = TARGETTING_TIMEOUT_SECS * 10; !hasTraveledDist() && timeout > 0; --timeout) {
        ros::spinOnce();    // Update odom readings
        clock();            // Continue movement
        wait.sleep();
    }
}
void TurtleMove::waitTargetRotation() {
    ros::Rate wait(10);
    for (int timeout = TARGETTING_TIMEOUT_SECS * 10; !hasTargetRotation() && timeout > 0; --timeout) {
        ros::spinOnce();    // Update odom readings
        clock();            // Continue movement
        wait.sleep();
    }
}
void TurtleMove::waitTimeSec(double time_secs) {
    ros::Rate wait(10);
    for (int time = 0; time < time_secs * 10; ++time) {
        clock();            // Continue movement
        wait.sleep();
    }
}