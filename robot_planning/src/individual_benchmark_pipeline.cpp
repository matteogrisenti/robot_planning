#include <ros/ros.h>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <set> 
#include <map>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <nav_msgs/Odometry.h> 
#include <algorithm> 
#include <mutex>
#include <chrono>
#include <sstream> // Aggiunto per stringstream

#include "map_library.h"
#include "roadmap.h"
#include "roadmap_factory.h"
#include "graph_search.h" 
#include "dubins_planner_client.h"
#include "planning_utils.h" 
#include "libraries/roadmap/roadmap_visualization.h" // NUOVO: Header per salvataggio immagini

// =============================================================================
// 1. CONFIGURAZIONE HYPERPARAMETERS
// =============================================================================
const double ROBOT_VELOCITY = 0.5;       
const double TURNING_RADIUS = 0.4;
const int MAX_DUBINS_RETRIES = 3;           
const double SAFETY_MARGIN = 0.90;          
const std::string OUTPUT_DIR = "src/robot_planning/src/test/";
const std::string METRICS_FILENAME = "benchmark_results_2.txt";

// =============================================================================
// 2. GLOBALS & STRUCTS
// =============================================================================
std::mutex odom_mutex;
double current_speed = 0.0;
Vertex current_pose_odom(0,0); 
bool odom_active = false;

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

// =============================================================================
// 3. HELPER FUNCTIONS
// =============================================================================

void appendMetricsToFile(const std::string& filepath, const RunMetrics& m) {
    std::ofstream file;
    file.open(filepath, std::ios_base::app); 
    
    if (file.is_open()) {
        file.seekp(0, std::ios::end);
        if (file.tellp() == 0) {
            // Header aggiornato con Score e Time Limit
            file << "Planner\tLimit(s)\tScore\tExec_T(s)\tPlan_T(s)\tVictims\tRetries\tSuccess\n";
        }
        
        file << m.planner << "\t" 
             << m.time_limit_set << "\t"
             << m.total_score << "\t"       
             << m.t_execution << "\t"       
             << m.t_total_planning << "\t"
             << m.victims_collected << "\t"
             << m.dubins_retries << "\t"
             << (m.success ? "YES" : "NO") << "\n";
             
        ROS_INFO("Metrics saved to: %s", filepath.c_str());
        file.close();
    } else {
        ROS_ERROR("Unable to open metrics file: %s", filepath.c_str());
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex);
    current_speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    current_pose_odom.x = msg->pose.pose.position.x;
    current_pose_odom.y = msg->pose.pose.position.y;
    odom_active = true;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}




// =============================================================================
// 4. MAIN
// =============================================================================
int main(int argc, char **argv) {
    ROS_WARN("=== BENCHMARK STARTED ===");

    // Standard ROS initialization with an AsyncSpinner to handle 
    // concurrent Odometry callbacks while the main thread is planning.
    ros::init(argc, argv, "individual_benchmark_node");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(2); 
    spinner.start();


    // --- ROS TOPIC SETUP ---
    // Subscribes to odometry to track real-time robot position and speed.
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);
    if (odom_sub.getNumPublishers() == 0) {
        odom_sub = nh.subscribe("/limo0/odom", 1, odomCallback);
    }

    // Publisher for debug visualization of the planned paths
    ros::Publisher debug_pub = nh.advertise<visualization_msgs::Marker>("/debug_path", 10); 
    DubinsClient dubins_client("/dubins_planner_server/follow_dubins_path");


    // --- DYNAMIC PARAMETER FETCHING ---
    // Allows changing planners (e.g., RRT, PRM) and time limits via command line.
    std::string planner_type;
    nh.param<std::string>("planner_type", planner_type, "acd");
    
    // Total time limit for the planning + execution phase
    double time_limit;
    nh.param<double>("time_limit", time_limit, 120.0); 
    
    ROS_INFO(">>> SELECTED PLANNER: %s", planner_type.c_str());
    ROS_INFO(">>> TIME LIMIT: %.1f s", time_limit);

    // --- ODOMETRY LOCK ---
    // Ensure we have a valid robot position before starting any planning.
    ROS_INFO("Waiting for Odometry...");
    while(ros::ok() && !odom_active) ros::Duration(0.1).sleep();
    
    Vertex startPose;
    {
        std::lock_guard<std::mutex> lock(odom_mutex);
        startPose = current_pose_odom;
    }
    ROS_INFO("Start Pose Locked: (%.2f, %.2f)", startPose.x, startPose.y);

    // Initialize metrics structure for final benchmark logging.
    RunMetrics metrics;
    metrics.planner = planner_type; 
    metrics.time_limit_set = time_limit;
    metrics.total_score = 0.0; // Init Score
    metrics.victims_collected = 0;
    metrics.dubins_retries = 0;
    metrics.success = false;

    try {
        // --- PHASE 1: ENVIRONMENT MODELING & ROADMAP ---
        // 1.1 Build the physical map (obstacles, victims, gates).
        map_builder::MapBuilder builder(nh, 1000.0);
        ROS_INFO("Building Map...");
        Map map = builder.buildMap();

        // 1.2 Generate the connectivity graph using the selected algorithm.
        ros::Time t_start_roadmap = ros::Time::now();
        std::shared_ptr<Roadmap> roadmap = generateRoadmap(planner_type, map);
        if (!roadmap) throw std::runtime_error("Roadmap generation failed.");
        
        // 1.3 Integration: Connect Start, Gate and Victims coordinates into the discrete roadmap graph.
        PlanningUtils::integratePosition(roadmap, startPose, map.obstacles.get_obstacles(), "Start");
        
        Vertex gatePose(0,0);
        if (!map.gates.get_gates().empty()) {
            Point g = map.gates.get_gates()[0].get_position();
            gatePose = Vertex(g.x, g.y);
            PlanningUtils::integratePosition(roadmap, gatePose, map.obstacles.get_obstacles(), "Gate");
        }

        // Map every victim into the graph to allow A* pathfinding between them.
        std::vector<Victim> victims = map.victims.get_victims();
        for(size_t i=0; i<victims.size(); ++i) {
            Point v = victims[i].get_center();
            PlanningUtils::integratePosition(roadmap, Vertex(v.x, v.y), map.obstacles.get_obstacles(), "Victim " + std::to_string(i));
        }

        // Create a lookup map to quickly reward the robot when a node is reached.
        std::map<int, double> victim_score_map;
        for(const auto& v : victims) {
            int idx = GraphSearch::TaskPlanner::getNearestNodeIdx(*roadmap, Vertex(v.get_center().x, v.get_center().y));
            if (idx != -1) {
                victim_score_map[idx] = v.get_radius();
            }
        }
        
        metrics.t_roadmap = (ros::Time::now() - t_start_roadmap).toSec();



        // --- PHASE 2: MISSION STRATEGY (TASK PLANNING) ---
        ros::Time t_start_plan = ros::Time::now();
        
        ROS_INFO("Planning Mission Sequence (Time Budget: %.1f s)...", time_limit);
        
        // Use a conservative velocity (85% of max) to account for time lost during turns.
        double conservative_velocity = ROBOT_VELOCITY * 0.85;
        
        // Sequence victims using a Greedy approach based on Value/Cost ratio.
        std::vector<int> missionSequence = GraphSearch::TaskPlanner::planMissionSequence(
            *roadmap, startPose, victims, gatePose, time_limit, conservative_velocity
        );
        
        if (missionSequence.empty()) throw std::runtime_error("Task Planning failed (Sequence empty).");



        // --- PHASE 3: GRAPH PLANNING ---
        // Expand the "Mission Sequence" (e.g., Start->Victim1->Gate) into a "Full Path" (node-by-node).
        std::set<int> criticalNodes(missionSequence.begin(), missionSequence.end());
        std::vector<int> fullGlobalPath;

        for (size_t i = 0; i < missionSequence.size() - 1; ++i) {
            std::vector<int> rawSegment = GraphSearch::AStarPlanner::computePath(*roadmap, missionSequence[i], missionSequence[i+1]);
            
            if (rawSegment.empty()) {
                ROS_ERROR("Path failed between nodes %d -> %d. Skipping segment.", missionSequence[i], missionSequence[i+1]);
                continue; 
            }

            // Shortcut the path to remove unnecessary zig-zags unless approaching a target.
            bool isCriticalApproach = (i == missionSequence.size() - 2);
            std::vector<int> segmentToAdd = isCriticalApproach ? rawSegment : 
                                                        PlanningUtils::optimizePath(rawSegment, *roadmap, map.obstacles.get_obstacles());

            if (fullGlobalPath.empty()) fullGlobalPath = segmentToAdd;
            else fullGlobalPath.insert(fullGlobalPath.end(), segmentToAdd.begin() + 1, segmentToAdd.end());
        }

        metrics.t_total_planning = (ros::Time::now() - t_start_plan).toSec();

        if (fullGlobalPath.empty()) throw std::runtime_error("Final Global Path is empty.");
        
        // Visualizzazione Debug Rviz
        GraphSearch::rviz_plan(fullGlobalPath, *roadmap, debug_pub);

        // Save a visual PNG snapshot of the final plan for the benchmark report.
        try {
            roadmap_viz::RoadmapVisualizer viz;
            viz.render(map, *roadmap);
            viz.drawPath(*roadmap, fullGlobalPath);
            std::stringstream ss;
            ss << OUTPUT_DIR << planner_type << "_limit_" << (int)time_limit << ".png";
            std::string img_path = ss.str();
            if (viz.saveToFile(img_path)) {
                ROS_INFO("SNAPSHOT SAVED: %s", img_path.c_str());
            } else {
                ROS_ERROR("SNAPSHOT FAILED: Could not save to %s", img_path.c_str());
            }
        } catch (const std::exception& e) {
            ROS_WARN("Visualization error: %s", e.what());
        }



        // --- PHASE 4: REAL-TIME EXECUTION ---
        ROS_INFO("Starting Execution...");
        ros::Time t_start_exec = ros::Time::now();
        bool mission_failed = false;

        for (size_t i = 0; i < fullGlobalPath.size() - 1; ++i) {
            int currentIdx = fullGlobalPath[i];
            int nextIdx = fullGlobalPath[i+1];
            const Vertex& currentV = roadmap->getVertex(currentIdx);
            const Vertex& targetV = roadmap->getVertex(nextIdx);

            // Determine if the next node requires a specific heading (theta)
            bool isCritical = (criticalNodes.find(nextIdx) != criticalNodes.end());
            double goal_theta = 0.0;
            double approach_angle = std::atan2(targetV.y - currentV.y, targetV.x - currentV.x);

            // Smooth heading: if next turn is shallow, point toward the exit of the node instead.
            if (!isCritical && i + 2 < fullGlobalPath.size()) {
                const Vertex& futureV = roadmap->getVertex(fullGlobalPath[i+2]);
                double exit_angle = std::atan2(futureV.y - targetV.y, futureV.x - targetV.x);
                if (std::abs(normalizeAngle(exit_angle - approach_angle)) <= (M_PI/3.0)) 
                    goal_theta = exit_angle;
            } else {
                goal_theta = approach_angle;
            }


            // --- NAVIGATION LOOP WITH STALL DETECTION ---
            bool already_at_target = false;
            if (isCritical) {
                double dist_to_target = 0.0;
                {
                    std::lock_guard<std::mutex> lock(odom_mutex);
                    dist_to_target = std::hypot(targetV.x - current_pose_odom.x, targetV.y - current_pose_odom.y);
                }
                if (dist_to_target < 0.50) {
                    ROS_WARN("Node %d (Critical) already reached. Skipping.", nextIdx);
                    already_at_target = true;
                }
            }

            int retry_count = 0;
            int current_max_retries = isCritical ? MAX_DUBINS_RETRIES : 1;
            bool node_success = already_at_target;

            while (!node_success && retry_count <= current_max_retries) {
                double v_cmd = (retry_count == 0) ? ROBOT_VELOCITY : (ROBOT_VELOCITY * 0.6); 
                
                dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, v_cmd, TURNING_RADIUS);
                
                ros::Rate loop_rate(5); 
                ros::Time stall_start_time = ros::Time::now();
                ros::Time segment_start_time = ros::Time::now(); 

                while(ros::ok()) {
                    if (dubins_client.waitForResult(0.01)) { 
                        if (dubins_client.isSuccess()) node_success = true;
                        break;
                    }

                    double v_now = 0.0;
                    { std::lock_guard<std::mutex> lock(odom_mutex); v_now = current_speed; }

                    if (ros::Time::now() - segment_start_time > ros::Duration(2.0)) { 
                        if (v_now > 0.02) stall_start_time = ros::Time::now();
                        else if (ros::Time::now() - stall_start_time > ros::Duration(3.0)) {
                            ROS_WARN("STALL DETECTED moving to node %d", nextIdx);
                            break; 
                        }
                    }
                    loop_rate.sleep();
                }

                if (node_success) break;
                
                retry_count++;
                metrics.dubins_retries++;
                
                if (retry_count <= current_max_retries) {
                    ROS_WARN("Retry %d/%d for Node %d", retry_count, current_max_retries, nextIdx);
                    ros::Duration(0.5).sleep();
                }
            }

            if (!node_success) {
                if (isCritical) {
                    ROS_ERROR("CRITICAL FAILURE at Node %d. Mission Aborted.", nextIdx);
                    mission_failed = true;
                    break; 
                } else {
                    ROS_WARN("Failed non-critical node %d. Skipping...", nextIdx);
                }
            } else if (isCritical && !already_at_target) {
                // SE è una vittima (non il Gate)
                if (i != fullGlobalPath.size() - 2) {
                    metrics.victims_collected++;
                    
                    // AGGIORNA PUNTEGGIO
                    if (victim_score_map.count(nextIdx)) {
                        double score_val = victim_score_map[nextIdx];
                        metrics.total_score += score_val;
                        ROS_INFO("VICTIM RESCUED! Value: %.1f | Total Score: %.1f", score_val, metrics.total_score);
                    } else {
                        ROS_WARN("Victim collected but score not found in map!");
                    }
                } else {
                    ROS_INFO("GATE REACHED!");
                }
                
                dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, 0.0, TURNING_RADIUS);
                ros::Duration(1.0).sleep();
            }
        }

        metrics.t_execution = (ros::Time::now() - t_start_exec).toSec();
        metrics.success = !mission_failed;

        ROS_INFO("\n=== MISSION REPORT ===");
        ROS_INFO("Planner: %s | Limit: %.1f s", metrics.planner.c_str(), metrics.time_limit_set);
        ROS_INFO("Total Score: %.1f", metrics.total_score);
        ROS_INFO("Time Exec: %.3fs", metrics.t_execution);
        ROS_INFO("Victims: %d | Success: %s", metrics.victims_collected, metrics.success ? "YES" : "NO");

        appendMetricsToFile(OUTPUT_DIR + METRICS_FILENAME, metrics);

    } catch (const std::exception& e) {
        ROS_ERROR("CRITICAL EXCEPTION: %s", e.what());
        return 1;
    }

    return 0;
}