#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <nav_msgs/Odometry.h> 
#include <visualization_msgs/Marker.h>

#include "map_library.h"
#include "libraries/roadmap.h"
#include "dubins_planner_client.h"

// Struct shared between Manager and Benchmark
struct RunMetrics {
    std::string planner;
    double time_limit_set;
    double total_score;     
    double t_roadmap;
    double t_total_planning;
    double t_execution;     
    int victims_collected;
    int dubins_retries;
    bool success;
};

class MissionManager {
public:
    MissionManager(ros::NodeHandle& nh);
    ~MissionManager() = default;

    /**
     * Executes the full pipeline: Map -> Roadmap -> Task Plan -> Execute
     * @param planner_type The string identifier for the algorithm (e.g., "prm", "rrt")
     * @param time_limit The total time budget for the mission
     * @param output_dir Directory to save debug images
     * @return RunMetrics containing the results of the run
     */
    RunMetrics run(const std::string& planner_type, double time_limit, const std::string& output_dir);

private:
    // ROS Handles
    ros::NodeHandle nh_;                            // Node Handle
    ros::Subscriber odom_sub_;                      // Odometry Subscriber
    ros::Publisher debug_pub_;                      // Debug Path Publisher
    std::unique_ptr<DubinsClient> dubins_client_;   // Dubins Action Client

    // Robot State
    std::mutex odom_mutex_;             // Mutex for odometry data
    double current_speed_;              // Current robot speed
    Vertex current_pose_odom_;          // Current robot pose
    bool odom_active_;                  // Flag indicating if odometry is active

    // Constants
    const double ROBOT_VELOCITY = 0.5;        
    const double TURNING_RADIUS = 0.4;
    const int MAX_DUBINS_RETRIES = 3;           

    // Helpers
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    double normalizeAngle(double angle);
};

#endif // MISSION_MANAGER_H