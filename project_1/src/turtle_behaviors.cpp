#include "turtle_behaviors.h"

#include <random>

// Helper to generate uniform random numbers
template<typename T>
T random(T range_from, T range_to) {
    std::random_device                  rand_dev;
    std::mt19937                        generator(rand_dev());
    std::uniform_int_distribution<T>    distr(range_from, range_to);
    return distr(generator);
}

TurtleBehaviors::TurtleBehaviors(ros::NodeHandle& nh, double linear_speed, double angular_speed)
     : move(nh, linear_speed, angular_speed) {
    this->linear_speed = linear_speed;
    this->angular_speed = angular_speed;
}

void TurtleBehaviors::halt() {
    move.stop();    // Halt

    ROS_INFO_STREAM("Halt");
    move.waitTimeSec(2);
    move.forwardDistHalt(HALT_REVERSE_DIST, true);
    move.waitTimeSec(2);
    escape();
}

void TurtleBehaviors::escape() {
    int randTurn = 180 + random(-ESCAPE_RAND_TURN, ESCAPE_RAND_TURN);
    
    move.stop();
    move.turnDegrees(randTurn);

    ROS_INFO_STREAM("Escape");
    move.waitTargetRotation();
}

// Turn away based on obstacle location left/right as -/+
void TurtleBehaviors::avoid(double obstacle_dir) {
    move.turnVelocity(AVOID_TURN_GAIN * angular_speed / obstacle_dir);
    ROS_INFO_STREAM("Avoid: " << obstacle_dir);
}

void TurtleBehaviors::drive() {
    if (move.hasTraveledDist()) {
        int randTurn = random(-DRIVE_RAND_TURN, DRIVE_RAND_TURN);
        
        move.forwardDist(DRIVE_TURN_INTERVAL);
        move.turnDegrees(randTurn);
        ROS_INFO_STREAM("Turning: " << randTurn);
    } else {
        move.forward();
    } 
}