#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

#include "turtle_move.h"

// Halt
static const double HALT_REVERSE_DIST = -0.5;   // Feet to back up when an obstacle is hit
// Escape
static const int ESCAPE_RAND_TURN = 30;         // Degrees of randomness while escaping
// Avoid
static const double AVOID_TURN_GAIN = 2;        // How quickly to turn while avoiding
// Drive
static const int DRIVE_RAND_TURN = 15;          // Degrees of randomness while driving
static const double DRIVE_TURN_INTERVAL = 1;    // Feet before random turn while driving

class TurtleBehaviors {
private:
    TurtleMove move;    // Control the robot
    double linear_speed, angular_speed;

public:
    TurtleBehaviors(ros::NodeHandle& nh, double linear_speed, double angular_speed);

    void halt();                        // Emergency stop
    void escape();                      // Turn 180 +/- 30 degrees
    void avoid(double obstacle_dir);    // Turn away from an obstacle, - on left, + on right
    void drive();                       // Drive forward, turn +/- 15 after 1ft

    void clock() { move.clock(); }      // Pass through to update movement
};
