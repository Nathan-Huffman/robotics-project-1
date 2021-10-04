#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>
#include <vector>

static const int SCAN_SAMPLE_MARGIN = 10;       // Number of points to sample on their side of idx
static const double SYMMETRIC_THRESHOLD = 1.15; // Max ratio allowed for symmetric approach

class TurtleSense {
private:
    class SenseListener{
    private:
        std::vector<double> raw_scan;
        int idx_min_scan_dist();
        double sample_near_idx(int idx, double backup);
    public:
        bool is_bumper_on;
        bool was_bumper_hit;
        bool symmetric_approach = true;
        double min_dist = DBL_MAX;
        double scan_bias = 0;
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& bump);
    public:
        SenseListener() {}
    };

    SenseListener listener;
    ros::Subscriber sub_laser;
    ros::Subscriber sub_bumper;

public:
    TurtleSense(ros::NodeHandle& nh);

    bool hasHitObstacle()               { return listener.was_bumper_hit; }
    void resetHitFlag()                 { listener.was_bumper_hit = false; }

    double nearestObstacleLocation()    { return listener.scan_bias; }
    double isSymmetricApproach()        { return listener.symmetric_approach; }

    double distToObstacleFeet()         { return listener.min_dist * 3.2808; }
    double distToObstacleMeters()       { return listener.min_dist; }
};