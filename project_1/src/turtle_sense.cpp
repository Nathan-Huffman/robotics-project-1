#include "turtle_sense.h"

int TurtleSense::SenseListener::idx_min_scan_dist() {
    return std::min_element(raw_scan.begin(), raw_scan.end()) - raw_scan.begin();
}

double TurtleSense::SenseListener::sample_near_idx(int idx, double backup) {
    double dist_sum = 0;    // Track sum total
    int num_samples = 0;    // Track num valid samples

    if (!isnan(raw_scan[idx])) {
        dist_sum = raw_scan[idx];
        num_samples = 1;
    }
    // Sensor fusion to the left of the index
    for (int scan_left = idx - 1; scan_left >= 0 && scan_left > idx-SCAN_SAMPLE_MARGIN; --scan_left) {
        if (!isnan(raw_scan[scan_left])) {
            dist_sum += raw_scan[scan_left];
            ++num_samples;
        }
    }
    // Sensor fusion to the right of the index
    for (int scan_right = idx - 1; scan_right < raw_scan.size() && scan_right < idx+SCAN_SAMPLE_MARGIN; ++scan_right) {
        if (!isnan(raw_scan[scan_right])) {
            dist_sum += raw_scan[scan_right];
            ++num_samples;
        }
    }
    if (num_samples == 0)
        return backup; // For the case where the laser is too close and has no valid values

    return dist_sum / num_samples;
}

//continuosly called to update LaserScan data
void TurtleSense::SenseListener::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    raw_scan.assign(std::begin(scan->ranges), std::end(scan->ranges));  // Store data from scan
    
    int idx_centerline = raw_scan.size() / 2;   // Get idx of straight ahead
    int idx_min_dist = idx_min_scan_dist();     // Get location of minimum element
    min_dist = raw_scan[idx_min_dist];          //Extract the minimum value
    if (isnan(min_dist))
        min_dist = scan->range_min;             // Account for obstacle being too close

    double center_sample = sample_near_idx(idx_centerline, scan->range_min);
    double closest_sample = sample_near_idx(idx_min_dist, scan->range_min); // Sample points around closest point
    double mirror_sample = sample_near_idx((idx_centerline - idx_min_dist) + idx_centerline - 1, scan->range_min);  // Mirror across 

    if (center_sample == scan->range_min) {
        symmetric_approach = true;      // Case for a head on collision where the obstacle is too close to see
        min_dist = scan->range_min;
        scan_bias = 0;
    }

    // Calculate how symmetric the obstacle appraoched is
    double symmetry_ratio = mirror_sample / (closest_sample + 0.0001);
    symmetric_approach = symmetry_ratio < SYMMETRIC_THRESHOLD;
     
     // Deduce where obstacle is left/right from -1/1
    scan_bias = ((idx_centerline - (double) idx_min_dist) / raw_scan.size()) * 2;
}

void TurtleSense::SenseListener::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& bump)
{
    is_bumper_on = bump->state==1;
    if (is_bumper_on)   // Bumper immediately rebounds, so save state
        was_bumper_hit = true;
}

// --------------------------------------

TurtleSense::TurtleSense(ros::NodeHandle& nh) {
    this->sub_laser = nh.subscribe("/scan", 30, &SenseListener::scanCallback, &listener);
    this->sub_bumper = nh.subscribe("/mobile_base/events/bumper", 10, &SenseListener::bumperCallback, &listener);
}