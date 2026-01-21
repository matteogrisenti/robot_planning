#include "mission_manager.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <set> 
#include <map>
#include <algorithm> 
#include <cmath> 

#include "libraries/roadmap_factory.h"
#include "libraries/graph_search.h" 
#include "libraries/planning_utils.h" 
#include "libraries/roadmap/roadmap_visualization.h"

// --- CONFIGURATION CONSTANTS ---
const double ROBOT_VELOCITY = 0.5;    
const double TURNING_RADIUS = 0.5;    
const int MAX_DUBINS_RETRIES = 3; 
const double MIN_EXECUTION_DIST = 1.5; 

MissionManager::MissionManager(ros::NodeHandle& nh) 
    : nh_(nh), current_speed_(0.0), current_pose_odom_(0,0), odom_active_(false) {
    
    // Topic Setup
    odom_sub_ = nh_.subscribe("/odom", 1, &MissionManager::odomCallback, this);
    if (odom_sub_.getNumPublishers() == 0) {
        odom_sub_ = nh_.subscribe("/limo0/odom", 1, &MissionManager::odomCallback, this);
    }
    
    debug_pub_ = nh_.advertise<visualization_msgs::Marker>("/debug_path", 10);
    dubins_client_ = std::make_unique<DubinsClient>("/dubins_planner_server/follow_dubins_path");
}

void MissionManager::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    current_pose_odom_.x = msg->pose.pose.position.x;
    current_pose_odom_.y = msg->pose.pose.position.y;
    odom_active_ = true;
}

double MissionManager::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

RunMetrics MissionManager::run(const std::string& planner_type, double time_limit, const std::string& output_dir) {
    RunMetrics metrics;
    metrics.planner = planner_type; 
    metrics.time_limit_set = time_limit;
    metrics.total_score = 0.0;
    metrics.victims_collected = 0;
    metrics.dubins_retries = 0;
    metrics.success = false;
    
    // Init timer values (Standard, no detailed breakdown to avoid compilation errors)
    metrics.t_roadmap = 0.0;
    metrics.t_total_planning = 0.0;
    metrics.t_execution = 0.0;

    // Wait for Odom
    ROS_INFO("[MissionManager] Waiting for Odometry...");
    while(ros::ok() && !odom_active_) ros::Duration(0.1).sleep();

    Vertex startPose(0,0);
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        startPose = current_pose_odom_;
    }
    ROS_INFO("[MissionManager] Start Pose Locked: (%.2f, %.2f)", startPose.x, startPose.y);

    try {
        ROS_INFO("[MissionManager] Started mission with planner '%s' and time limit %.1f s", planner_type.c_str(), time_limit);
        
        // --- PHASE 1: ENVIRONMENT MODELING & ROADMAP ---
        map_builder::MapBuilder builder(nh_, 1000.0);
        ROS_INFO("[MissionManager] Building Map...");
        Map map = builder.buildMap();

        ros::Time t_start_roadmap = ros::Time::now();
        std::shared_ptr<Roadmap> roadmap = generateRoadmap(planner_type, map);
        if (!roadmap) throw std::runtime_error("[MissionManager] Roadmap generation failed.");
        
        // Integrate Start
        PlanningUtils::integratePosition(roadmap, startPose, map.obstacles.get_obstacles(), "Start");
        
        // Integrate Gate
        Vertex gatePose(0,0);
        if (!map.gates.get_gates().empty()) {
            Point g = map.gates.get_gates()[0].get_position();
            gatePose = Vertex(g.x, g.y);
            PlanningUtils::integratePosition(roadmap, gatePose, map.obstacles.get_obstacles(), "Gate");
        }
        
        // Integrate Victims & Build Score Map
        std::vector<Victim> victims = map.victims.get_victims();    
        std::map<int, double> victim_score_map;                     
        
        for(size_t i=0; i<victims.size(); ++i) {
            Point center = victims[i].get_center();
            PlanningUtils::integratePosition(roadmap, Vertex(center.x, center.y), map.obstacles.get_obstacles(), "Victim " + std::to_string(i));
            
            Victim v = victims[i];
            int idx = GraphSearch::TaskPlanner::getNearestNodeIdx(*roadmap, Vertex(Vertex(v.get_center().x, v.get_center().y)));
            if (idx != -1) victim_score_map[idx] = v.get_radius(); 
        }
        metrics.t_roadmap = (ros::Time::now() - t_start_roadmap).toSec();


        // --- PHASE 2 & 3: TASK PLANNING & PATH FINDING ---
        ros::Time t_start_plan = ros::Time::now(); 
        ROS_INFO("[MissionManager] Planning Mission Sequence...");
        
        double conservative_velocity = ROBOT_VELOCITY * 0.90;
        std::vector<int> missionSequence = GraphSearch::TaskPlanner::planMissionSequence(
            *roadmap, startPose, victims, gatePose, time_limit, conservative_velocity
        );
        
        if (missionSequence.empty()) throw std::runtime_error("[MissionManager] Task Planning failed (Sequence empty).");

        ROS_INFO("[MissionManager] Generating Detailed Path...");
        std::vector<int> fullGlobalPath;
        
        if (missionSequence.size() >= 2) {
            for (size_t i = 0; i < missionSequence.size() - 1; ++i) {
                
                // 3.1 GRAPH SEARCH (A*)
                std::vector<int> rawSegment = GraphSearch::AStar::computePath(*roadmap, missionSequence[i], missionSequence[i+1]);

                if (rawSegment.empty()) {
                    ROS_ERROR("Path failed between nodes %d -> %d.", missionSequence[i], missionSequence[i+1]);
                    continue; 
                }

                // 3.2 PATH OPTIMIZATION
                bool isCriticalApproach = (i == missionSequence.size() - 2);
                std::vector<int> segmentToAdd;
                
                if (isCriticalApproach) {
                    segmentToAdd = rawSegment; 
                } else {
                    segmentToAdd = PlanningUtils::optimizePath(rawSegment, *roadmap, map.obstacles.get_obstacles());
                }

                // Stitching
                if (fullGlobalPath.empty()) {
                    fullGlobalPath = segmentToAdd;
                } else {
                    fullGlobalPath.insert(fullGlobalPath.end(), segmentToAdd.begin() + 1, segmentToAdd.end());
                }
            }
        }

        if (fullGlobalPath.empty()) throw std::runtime_error("[MissionManager] Final Global Path is empty.");

        // TOTAL PLANNING TIME
        metrics.t_total_planning = (ros::Time::now() - t_start_plan).toSec();

        // Debug Visualization
        GraphSearch::rviz_plan(fullGlobalPath, *roadmap, debug_pub_);

        // Save Snapshot
        try {
            roadmap_viz::RoadmapVisualizer viz;
            viz.render(map, *roadmap);
            viz.drawPath(*roadmap, fullGlobalPath);
            std::stringstream ss;
            ss << output_dir << planner_type << "_limit_" << (int)time_limit << ".png";
            viz.saveToFile(ss.str());
        } catch (...) { ROS_WARN("[MissionManager] Visualization save failed."); }


        // --- PHASE 4: REAL-TIME EXECUTION ---
        ROS_INFO("[MissionManager] Starting Execution...");
        ros::Time t_start_exec = ros::Time::now();
        bool mission_failed = false;

        // --- PATH PRUNING ---
        // Filters intermediate nodes before execution starts.
        std::vector<int> execPath;
        if (!fullGlobalPath.empty()) {
            execPath.push_back(fullGlobalPath[0]); // Keep Start
            int lastKeptIdx = fullGlobalPath[0];
            
            // Fast lookup for critical targets
            std::set<int> targetsSet(missionSequence.begin(), missionSequence.end());

            for (size_t k = 1; k < fullGlobalPath.size(); ++k) {
                int currIdx = fullGlobalPath[k];
                
                bool isTarget = (targetsSet.find(currIdx) != targetsSet.end());
                bool isLastNode = (k == fullGlobalPath.size() - 1);
                
                if (isTarget || isLastNode) {
                    execPath.push_back(currIdx);
                    lastKeptIdx = currIdx;
                } else {
                    const Vertex& vLast = roadmap->getVertex(lastKeptIdx);
                    const Vertex& vCurr = roadmap->getVertex(currIdx);
                    double dist = std::hypot(vCurr.x - vLast.x, vCurr.y - vLast.y);
                    
                    if (dist >= MIN_EXECUTION_DIST) {
                        execPath.push_back(currIdx);
                        lastKeptIdx = currIdx;
                    }
                }
            }
        }
        ROS_INFO("[MissionManager] Path Pruned: %lu -> %lu nodes (Min Dist: %.1fm)", 
                 fullGlobalPath.size(), execPath.size(), MIN_EXECUTION_DIST);

        // --- EXECUTION LOOP ---
        size_t currentTargetSeqIdx = 1; 

        for (size_t i = 0; i < execPath.size() - 1; ++i) {
            int currentIdx = execPath[i];
            int nextIdx = execPath[i+1];
            const Vertex& currentV = roadmap->getVertex(currentIdx);
            const Vertex& targetV = roadmap->getVertex(nextIdx);

            bool isCurrentMissionTarget = (currentTargetSeqIdx < missionSequence.size() && 
                                           nextIdx == missionSequence[currentTargetSeqIdx]);
            
            // Check if this is the final segment of the entire path
            bool isEndOfPath = (i == execPath.size() - 2);
            bool isFinalGate = (isCurrentMissionTarget && currentTargetSeqIdx == missionSequence.size() - 1);
            
            // Safety: Force final gate behavior if we are at the end of the pruned path
            if (isEndOfPath) isFinalGate = true;

            double goal_theta = 0.0;
            double approach_angle = std::atan2(targetV.y - currentV.y, targetV.x - currentV.x);

            // Lookahead
            if (!isCurrentMissionTarget && i + 2 < execPath.size()) {
                const Vertex& futureV = roadmap->getVertex(execPath[i+2]);
                double dist_to_future = std::hypot(futureV.x - targetV.x, futureV.y - targetV.y);
                
                if (dist_to_future > 2.0) {
                    double exit_angle = std::atan2(futureV.y - targetV.y, futureV.x - targetV.x);
                    if (std::abs(normalizeAngle(exit_angle - approach_angle)) <= (M_PI/3.0)) 
                        goal_theta = exit_angle;
                    else 
                        goal_theta = approach_angle;
                } else {
                    goal_theta = approach_angle;
                }
            } else {
                goal_theta = approach_angle;
            }

            // Retry Loop
            int retry_count = 0;
            int current_max_retries = isCurrentMissionTarget ? MAX_DUBINS_RETRIES : 1;
            bool node_success = false;

            while (!node_success && retry_count <= current_max_retries) {
                double v_cmd = ROBOT_VELOCITY;
                if (retry_count > 0) v_cmd = ROBOT_VELOCITY * 0.8; 

                dubins_client_->sendGoal(targetV.x, targetV.y, goal_theta, v_cmd, TURNING_RADIUS);
                
                ros::Rate loop_rate(10); 
                ros::Time stall_start_time = ros::Time::now();
                ros::Time segment_start_time = ros::Time::now(); 

                while(ros::ok()) {
                    if (dubins_client_->waitForResult(0.05)) { 
                        if (dubins_client_->isSuccess()) node_success = true;
                        break;
                    }
                    
                    // Stall Detection
                    double v_now = 0.0;
                    { std::lock_guard<std::mutex> lock(odom_mutex_); v_now = current_speed_; }
                    if (ros::Time::now() - segment_start_time > ros::Duration(1.0)) { 
                        if (v_now > 0.05) stall_start_time = ros::Time::now();
                        else if (ros::Time::now() - stall_start_time > ros::Duration(3.0)) {
                            ROS_WARN("[MissionManager] STALL DETECTED node %d", nextIdx);
                            break; 
                        }
                    }
                    loop_rate.sleep();
                }

                if (node_success) break;
                
                retry_count++;
                metrics.dubins_retries++;
                if (retry_count <= current_max_retries) ros::Duration(0.1).sleep();
            }

            if (!node_success) {
                if (isCurrentMissionTarget) {
                    ROS_ERROR("[MissionManager] CRITICAL FAILURE at Target Node %d. Aborting.", nextIdx);
                    mission_failed = true;
                    break; 
                }
            } else {
                if (isCurrentMissionTarget) {
                    if (!isFinalGate) {
                        metrics.victims_collected++;
                        if (victim_score_map.count(nextIdx)) {
                            metrics.total_score += victim_score_map[nextIdx];
                            ROS_INFO("[MissionManager] Victim Collected at Node %d. Score: %.1f", nextIdx, metrics.total_score);
                        }
                    }
                    
                    currentTargetSeqIdx++;

                    if (isFinalGate) {
                        ROS_INFO("[MissionManager] Final Gate Reached. Mission Complete.");
                        ros::Duration(2.0).sleep();
                    }
                }
            }
        }

        metrics.t_execution = (ros::Time::now() - t_start_exec).toSec();
        metrics.success = !mission_failed;

    } catch (const std::exception& e) {
        ROS_ERROR("[MissionManager] Mission Manager Exception: %s", e.what());
        metrics.success = false;
    }

    return metrics;
}