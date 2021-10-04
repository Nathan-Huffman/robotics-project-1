#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>

static const double ANGULAR_MAX_VEL = 2.0;                  // Clip the max turning sent to the robot
static const double ANGULAR_TARGETTING_GAIN = 30;           // Proportional control for angle targetting
static const double ANGULAR_TARGETTING_THRESHOLD = 0.05;    // Threshold for reaching target angle
static const double ANGULAR_UPDATE_THRESHOLD = 1.0;         // Limit how frequently to change targetting rotation
static const int TARGETTING_TIMEOUT_SECS = 120;             // Timeout to wait to target reached

class TurtleMove {
private:
    class MoveListener{    
    private:
        bool is_valid = false;
    public:
        geometry_msgs::Pose2D current_pose;
        void odomCallback(const nav_msgs::OdometryConstPtr& msg);
        bool ensure_valid();
    };

    double linear_velocity_default, angular_velocity_default;    
    
    bool is_targetting_angle = false;   // Flag to trigger angle targetting
    bool halt_at_target_dist = false;   // Flag to stop once target distance hit
    double target_angle;                // Hold the target angle
    double current_angle_diff;          // Hold the difference between current and target angle
    double target_distance   = 0;       // Hold the target distance
    double traveled_distance = 0;       // Hold the traveled distance
    geometry_msgs::Pose2D checkpoint;   // Used for traveling a specified distance
    
    MoveListener listener;              // Hold info from odometry
    ros::Subscriber sub_odometry;       // Subscribe to odometry
    ros::Publisher pub_cmd_vel;         // Publish commands to control the robot
    geometry_msgs::Twist base_cmd;      // Reused var to hold cmd to send

public:
    TurtleMove(ros::NodeHandle& nh, double linear_velocity, double angular_velocity);

    void clock();  // Publishes msg to move robot as desired

    void forward()      { forwardVelocity(linear_velocity_default); }
    void forwardVelocity(double velocity);
    void forwardDist(double distFeet)   { forwardDistVelocity(distFeet, linear_velocity_default); };
    void forwardDistHalt(double distFeet, bool halt) { forwardDistVelocityHalt(distFeet, linear_velocity_default, halt); }
    void forwardDistVelocity(double distFeet, double velocity) { forwardDistVelocityHalt(distFeet, velocity, false); }
    void forwardDistVelocityHalt(double distFeet, double velocity, bool halt);
    void forwardStop();

    void turnLeft()     { turnVelocity(-angular_velocity_default); }
    void turnRight()    { turnVelocity(angular_velocity_default); }
    void turnVelocity(double velocity);
    void turnDegrees(double angle);
    void turnStop();
    
    void stop();

    bool hasTraveledDist();     // Return status of target
    bool hasTargetRotation();

    void waitTraveledDist();    // Pause until status reached
    void waitTargetRotation();
    void waitTimeSec(double time_secs);
};