#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <mutex>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <set>

// Librerie del progetto
#include "map_library.h"
#include "roadmap.h"
#include "roadmap_factory.h"
#include "astar.h"
#include "dubins_planner_client.h"
#include "planning_utils.h"

// --- CONFIGURAZIONE HYPERPARAMETERS ---
// PLANNER TYPE: "acd", "ecd", "mcr", "spr", "prm", "rrt", "rrt_star"
const std::string PLANNER_TYPE = "prm";     // HYPERPARAMETER: Planner to test
const double ROBOT_VELOCITY = 0.5;       
const double TURNING_RADIUS = 0.4;
const int MAX_DUBINS_RETRIES = 3;           // HYPERPARAMETER: Max retries for critical Dubins nodes
const double SAFETY_MARGIN = 0.90;          // HYPERPARAMETER: Minimum clearance from obstacles in meters

std::string output_dir = "src/robot_planning/src/test/";

// --- RESULT STRUCTURE ---
struct BenchmarkResult {
    std::string method;
    double roadmap_time_ms;
    double planning_time_ms;
    double execution_time_sec;
    int victims_collected;
    int dubins_retries;
    bool success;
    std::string notes;
};


// --- GLOBALS ---
std::mutex odom_mutex;
double current_speed = 0.0;
Vertex current_pose_odom(0, 0);
double current_yaw_odom = 0.0;
bool odom_active = false;
Vertex INITIAL_START_POSE(0, 0);
bool initial_pose_captured = false;


// --- ODOMETRY CALLBACK ---
// update current robot pose and speed
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex);   // Lock mutex for thread safety

    // extract speed and position
    current_speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    current_pose_odom.x = msg->pose.pose.position.x;
    current_pose_odom.y = msg->pose.pose.position.y;
    
    // extract orientation
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    // pass from quaternion to euler angles
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_odom);    
    
    // Lock initial position ( first callback only )
    if (!initial_pose_captured) {
        INITIAL_START_POSE = current_pose_odom;
        initial_pose_captured = true;
        ROS_INFO(">>> START POSITION LOCKED: (%.2f, %.2f)", INITIAL_START_POSE.x, INITIAL_START_POSE.y);
    }

    // Set odom active
    odom_active = true;
}





int main(int argc, char** argv) {
    ros::init(argc, argv, "individual_benchmark_node");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);
    if (odom_sub.getNumPublishers() == 0)
        odom_sub = nh.subscribe("/limo0/odom", 1, odomCallback);
    
    ros::Publisher debug_pub = nh.advertise<visualization_msgs::Marker>("/debug_path", 10);
    DubinsClient dubins_client("/dubins_planner_server/follow_dubins_path");
    
    // Attesa Odometry
    ROS_INFO("Waiting for Odometry...");
    while (ros::ok() && !odom_active) {
        ros::Duration(0.1).sleep();
    }
    
    // Costruzione Mappa
    map_builder::MapBuilder builder(nh, 1000.0);
    ROS_INFO("Building Map...");
    Map map = builder.buildMap();
    
    ROS_WARN("\n=== BENCHMARK: %s ===", PLANNER_TYPE.c_str());
    
    BenchmarkResult result;
    result.method = PLANNER_TYPE;
    result.dubins_retries = 0;
    result.victims_collected = 0;
    result.success = false;
    
    std::shared_ptr<Roadmap> roadmap = nullptr;
    
    try {
        // A. ROADMAP GENERATION
        auto t1 = std::chrono::high_resolution_clock::now();
        roadmap = generateRoadmap(PLANNER_TYPE, map);
        
        // Integrate Start Position into Roadmap
        PlanningUtils::integratePosition(roadmap, INITIAL_START_POSE, map.obstacles.get_obstacles(), "Start");
        
        // Integrate Gate Position into Roadmap
        Vertex gatePose(0, 0);
        if (!map.gates.get_gates().empty()) {
            Point g = map.gates.get_gates()[0].get_position();
            gatePose = Vertex(g.x, g.y);
            PlanningUtils::integratePosition(roadmap, gatePose, map.obstacles.get_obstacles(), "Gate");
        }
        
        // Integrate Victims Positions into Roadmap
        std::vector<Victim> victims = map.victims.get_victims();
        for (size_t i = 0; i < victims.size(); ++i) {
            Point v = victims[i].get_center();
            PlanningUtils::integratePosition(roadmap, Vertex(v.x, v.y), map.obstacles.get_obstacles(), "Victim" + std::to_string(i));
        }
        
        result.roadmap_time_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t1).count();
        ROS_INFO("Roadmap built: %d vertices, %.0f ms", roadmap->getNumVertices(), result.roadmap_time_ms);


        // B. PLANNING
        auto t3 = std::chrono::high_resolution_clock::now();

        // Determine mission sequence: the list of victims and gate to visit
        std::vector<int> missionSequence = GraphSearch::TaskPlanner::planMissionSequence(*roadmap, INITIAL_START_POSE, victims, gatePose);
        std::set<int> criticalNodes(missionSequence.begin(), missionSequence.end());
        
        // Compute full path by connecting each critical waypoint in the mission sequence
        std::vector<int> fullGlobalPath;
        for (size_t i = 0; i < missionSequence.size() - 1; ++i) {

            // Compute path segment between two critical waypoints using A*
            std::vector<int> rawSegment = GraphSearch::AStarPlanner::computePath(*roadmap, missionSequence[i], missionSequence[i + 1]);
            if (rawSegment.empty()) continue;
            
            // Optimize non-critical segments of the path
            // NB: the critical segment is the approach to the gate (last segment)
            bool isCriticalApproach = (i == missionSequence.size() - 2);
            std::vector<int> segmentToAdd = isCriticalApproach ? rawSegment : PlanningUtils::optimizePath(rawSegment, *roadmap, map.obstacles.get_obstacles());
            
            // Append segment to full path
            if (fullGlobalPath.empty())
                fullGlobalPath = segmentToAdd;
            else
                fullGlobalPath.insert(fullGlobalPath.end(), segmentToAdd.begin() + 1, segmentToAdd.end());
        }
        
        result.planning_time_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t3).count();
        
        if (fullGlobalPath.empty()) throw std::runtime_error("Path Empty.");
        
        GraphSearch::rviz_plan(fullGlobalPath, *roadmap, debug_pub);
        ROS_INFO("Mission planned: %lu waypoints, %.0f ms", fullGlobalPath.size(), result.planning_time_ms);
        
        // Visualize Roadmap and Path
        roadmap_viz::RoadmapVisualizer viz;
        viz.render(map, *roadmap);
        viz.drawPath(*roadmap, fullGlobalPath);
        if (viz.saveToFile(output_dir + "benchmark_" + PLANNER_TYPE + "_path.png")) {
            ROS_INFO("DEBUG: Saved roadmap and path image");
        }else {
            ROS_WARN("DEBUG: Failed to save roadmap image");
        }

        // C. EXECUTION
        ROS_INFO("Executing Mission...");
        auto t5 = std::chrono::high_resolution_clock::now();
        
        // Execute each segment of the path
        for (size_t i = 0; i < fullGlobalPath.size() - 1; ++i) {
            int nextIdx = fullGlobalPath[i + 1];
            const Vertex& currentV = roadmap->getVertex(fullGlobalPath[i]);
            const Vertex& targetV = roadmap->getVertex(nextIdx);

            // Check if the target node is critical (victim or gate)
            bool isCritical = (criticalNodes.find(nextIdx) != criticalNodes.end());
            
            double goal_theta = std::atan2(targetV.y - currentV.y, targetV.x - currentV.x);
            
            int retry_count = 0;
            bool node_success = false;

            // Retry loop for critical steps. Can happen that Dubins planner fails due to local obstacles
            while (!node_success && retry_count <= (isCritical ? MAX_DUBINS_RETRIES : 1)) {
                // Send goal to Dubins planner server
                dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, ROBOT_VELOCITY, TURNING_RADIUS);
                
                ros::Time start_t = ros::Time::now();
                ros::Time stall_t = ros::Time::now();
                while (ros::ok()) {
                    if (dubins_client.waitForResult(0.02)) {
                        if (dubins_client.isSuccess()) node_success = true;
                        break;
                    }
                    
                    double v;
                    {
                        std::lock_guard<std::mutex> lock(odom_mutex);
                        v = current_speed;
                    }
                    if (v > 0.05) stall_t = ros::Time::now();
                    if (ros::Time::now() - stall_t > ros::Duration(3.0)) break;
                }
                
                if (node_success) break;
                retry_count++;
                result.dubins_retries++;
            }
            
            if (!node_success && isCritical) break;
            
            // If critical node reached, simulate victim collection
            if (isCritical) {
                result.victims_collected++;
                dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, 0.0, TURNING_RADIUS);
                ros::Duration(0.5).sleep();
            }
        }
        
        result.execution_time_sec = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t5).count();
        if (result.victims_collected > 0) result.success = true;
        
    } catch (const std::exception& e) {
        ROS_ERROR("Benchmark Failed: %s", e.what());
        result.notes = e.what();
    }
    
    // Ritorno finale
    ROS_INFO("Mission Ended. ");
    
    // REPORT
    std::cout << "\n\n=== BENCHMARK RESULTS ===\n";
    std::cout << "Planner: " << result.method << "\n";
    std::cout << "Roadmap Time: " << result.roadmap_time_ms << " ms\n";
    std::cout << "Planning Time: " << result.planning_time_ms << " ms\n";
    std::cout << "Execution Time: " << result.execution_time_sec << " sec\n";
    std::cout << "Victims Collected: " << result.victims_collected << "\n";
    std::cout << "Dubins Retries: " << result.dubins_retries << "\n";
    std::cout << "Success: " << (result.success ? "YES" : "NO") << "\n";
    if (!result.notes.empty()) std::cout << "Notes: " << result.notes << "\n";
    std::cout << "========================\n";
    
    return 0;
}