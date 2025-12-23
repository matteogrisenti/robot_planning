#include <ros/ros.h>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <set> 
#include <map>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/PoseWithCovarianceStamped.h> 
#include <algorithm> 
#include <chrono> 
#include <tf/transform_datatypes.h>

// Librerie del progetto
#include "map_library.h"
#include "roadmap.h"
#include "roadmap_factory.h"
#include "astar.h" 
#include "dubins_planner_client.h"
#include "planning_utils.h" 

// --- CONFIGURAZIONE ---
const double ROBOT_VELOCITY = 0.5;
const double TURNING_RADIUS = 0.4;
const std::vector<std::string> PLANNERS_TO_TEST = {"acd", "ecd", "mcr", "spr", "prm", "rrt", "rrt_star"};

// --- STRUTTURA RISULTATI ---
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
Vertex current_pose_odom(0,0); 
double current_yaw_odom = 0.0;
bool odom_active = false;
Vertex INITIAL_START_POSE(0,0);
bool initial_pose_captured = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex);
    current_speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    current_pose_odom.x = msg->pose.pose.position.x;
    current_pose_odom.y = msg->pose.pose.position.y;
    
    tf::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_odom);

    if (!initial_pose_captured) {
        INITIAL_START_POSE = current_pose_odom;
        initial_pose_captured = true;
        ROS_INFO(">>> START POSITION LOCKED: (%.2f, %.2f)", INITIAL_START_POSE.x, INITIAL_START_POSE.y);
    }
    odom_active = true;
}

// --- HELPER DI PIANIFICAZIONE (Standard) ---
bool isSegmentSafe(const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles, double min_clearance) {
    if (PlanningUtils::lineSegmentIntersectsObstacle(p1, p2, obstacles)) return false;
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    int steps = std::max(2, (int)(dist / 0.05)); 
    for (int i = 0; i <= steps; ++i) {
        double t = (double)i / steps;
        Vertex p(p1.x + t*(p2.x - p1.x), p1.y + t*(p2.y - p1.y));
        if (PlanningUtils::distanceToNearestObstacle(p, obstacles) < min_clearance) return false; 
    }
    return true;
}

std::vector<int> optimizePath(const std::vector<int>& rawPath, const Roadmap& roadmap, const std::vector<Obstacle>& obstacles) {
    if (rawPath.size() < 2) return rawPath;
    std::vector<int> optimized;
    optimized.push_back(rawPath[0]);
    int currentIdx = 0;
    while (currentIdx < rawPath.size() - 1) {
        bool shortcutFound = false;
        for (int i = rawPath.size() - 1; i > currentIdx + 1; --i) {
            const Vertex& vStart = roadmap.getVertex(rawPath[currentIdx]);
            const Vertex& vEnd = roadmap.getVertex(rawPath[i]);
            if (isSegmentSafe(vStart, vEnd, obstacles, 0.90)) {
                optimized.push_back(rawPath[i]);
                currentIdx = i;
                shortcutFound = true;
                break;
            }
        }
        if (!shortcutFound) {
            optimized.push_back(rawPath[currentIdx + 1]);
            currentIdx++;
        }
    }
    if (optimized.size() > 2) {
        std::vector<int> filtered;
        filtered.push_back(optimized[0]);
        for (size_t i = 1; i < optimized.size() - 1; ++i) {
            const Vertex& prev = roadmap.getVertex(filtered.back());
            const Vertex& curr = roadmap.getVertex(optimized[i]);
            if (std::hypot(curr.x - prev.x, curr.y - prev.y) > 1.0) {
                filtered.push_back(optimized[i]);
            }
        }
        filtered.push_back(optimized.back());
        return filtered;
    }
    return optimized;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void integratePosition(std::shared_ptr<Roadmap>& roadmap, const Vertex& pos, const std::vector<Obstacle>& obstacles, const std::string& label) {
    for(int i=0; i<roadmap->getNumVertices(); ++i) {
        if(roadmap->getVertex(i).distance(pos) < 0.05) return; 
    }
    int newIdx = roadmap->addVertex(pos);
    double search_radius = 15.0; 
    std::vector<std::pair<double, int>> neighbors;
    for(int i=0; i<roadmap->getNumVertices(); ++i) {
        if(i == newIdx) continue;
        double d = roadmap->getVertex(i).distance(pos);
        if(d < search_radius) neighbors.push_back({d, i});
    }
    std::sort(neighbors.begin(), neighbors.end());
    std::vector<double> margins = {0.90, 0.60, 0.30}; 
    int connected_count = 0;
    for (double margin : margins) {
        if (connected_count >= 3) break;
        for(const auto& pair : neighbors) {
            if (connected_count >= 15) break; 
            int targetIdx = pair.second;
            bool edgeExists = false;
            for(const auto& e : roadmap->getEdges(newIdx)) if(e.targetVertex == targetIdx) edgeExists = true;
            if(edgeExists) continue;
            bool possible = false;
            if (margin > 0.0) possible = isSegmentSafe(pos, roadmap->getVertex(targetIdx), obstacles, margin);
            else possible = !PlanningUtils::lineSegmentIntersectsObstacle(pos, roadmap->getVertex(targetIdx), obstacles);
            if(possible) {
                roadmap->addEdge(newIdx, targetIdx, pair.first);
                roadmap->addEdge(targetIdx, newIdx, pair.first); 
                connected_count++;
            }
        }
    }
}

// --- FUNZIONI DI GESTIONE AUTOMATICA ---

void clearMap(ros::Publisher& pub) {
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.action = 3; // DELETEALL
    pub.publish(m);
    ros::spinOnce();
}

// Guida il robot indietro allo start usando Dubins Planner
void driveBackToStart(DubinsClient& client, const Map& map, std::shared_ptr<Roadmap> currentRoadmap) {
    Vertex currentPose;
    {
        std::lock_guard<std::mutex> lock(odom_mutex);
        currentPose = current_pose_odom;
    }
    double dist = std::hypot(currentPose.x - INITIAL_START_POSE.x, currentPose.y - INITIAL_START_POSE.y);
    if (dist < 0.5) return; // Già allo start

    ROS_INFO(">>> AUTO-RETURN INITIATED (Dist: %.2fm)", dist);

    std::shared_ptr<Roadmap> returnRoadmap = currentRoadmap;
    
    // Se la roadmap corrente è nulla o vuota (es. fallimento), ne generiamo una di emergenza (PRM)
    if (!returnRoadmap || returnRoadmap->getNumVertices() < 2) {
        ROS_WARN("Generating Emergency PRM for return trip...");
        returnRoadmap = generateRoadmap("prm", map);
    }

    // Integriamo Posizione Corrente e Start nella roadmap per trovare il percorso
    integratePosition(returnRoadmap, currentPose, map.obstacles.get_obstacles(), "ReturnStart");
    integratePosition(returnRoadmap, INITIAL_START_POSE, map.obstacles.get_obstacles(), "ReturnEnd");

    // Troviamo gli indici
    int startIdx = -1, endIdx = -1;
    double minD1 = 1e9, minD2 = 1e9;
    for(int i=0; i<returnRoadmap->getNumVertices(); ++i) {
        double d1 = returnRoadmap->getVertex(i).distance(currentPose);
        if(d1 < minD1) { minD1 = d1; startIdx = i; }
        double d2 = returnRoadmap->getVertex(i).distance(INITIAL_START_POSE);
        if(d2 < minD2) { minD2 = d2; endIdx = i; }
    }

    if (startIdx == -1 || endIdx == -1) {
        ROS_ERROR("Failed to integrate points for return. Attempting blind move.");
        client.sendGoal(INITIAL_START_POSE.x, INITIAL_START_POSE.y, 0.0, ROBOT_VELOCITY, TURNING_RADIUS);
        ros::Duration(5.0).sleep();
        return;
    }

    std::vector<int> path = GraphSearch::AStarPlanner::computePath(*returnRoadmap, startIdx, endIdx);
    if (path.empty()) {
        ROS_ERROR("No return path found! Driving blindly to start.");
        client.sendGoal(INITIAL_START_POSE.x, INITIAL_START_POSE.y, 0.0, ROBOT_VELOCITY, TURNING_RADIUS);
    } else {
        path = optimizePath(path, *returnRoadmap, map.obstacles.get_obstacles());
        ROS_INFO("Executing Return Path (%lu nodes)...", path.size());
        
        for (size_t i = 0; i < path.size() - 1; ++i) {
            Vertex target = returnRoadmap->getVertex(path[i+1]);
            Vertex curr = returnRoadmap->getVertex(path[i]);
            double angle = std::atan2(target.y - curr.y, target.x - curr.x);
            
            client.sendGoal(target.x, target.y, angle, ROBOT_VELOCITY * 0.8, TURNING_RADIUS);
            
            // Wait loop semplificato per il ritorno
            ros::Time start_t = ros::Time::now();
            while(ros::ok()) {
                if (client.waitForResult(0.05)) break;
                if (ros::Time::now() - start_t > ros::Duration(10.0)) break; // Timeout per segmento
            }
        }
    }

    // Correzione finale dell'orientamento allo start
    client.sendGoal(INITIAL_START_POSE.x, INITIAL_START_POSE.y, 0.0, 0.0, TURNING_RADIUS);
    ros::Duration(1.0).sleep();
    
    // Verifica finale
    {
        std::lock_guard<std::mutex> lock(odom_mutex);
        dist = std::hypot(current_pose_odom.x - INITIAL_START_POSE.x, current_pose_odom.y - INITIAL_START_POSE.y);
    }
    if (dist < 0.8) ROS_INFO(">>> ROBOT RETURNED TO START.");
    else ROS_WARN(">>> RETURN INCOMPLETE (Dist: %.2fm). Proceeding anyway.", dist);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "benchmark_pipeline_node");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);
    if (odom_sub.getNumPublishers() == 0) odom_sub = nh.subscribe("/limo0/odom", 1, odomCallback);
    ros::Publisher debug_pub = nh.advertise<visualization_msgs::Marker>("/debug_path", 10);
    DubinsClient dubins_client("/dubins_planner_server/follow_dubins_path");

    // 1. Attesa Start
    ROS_INFO("Waiting for Odometry...");
    while(ros::ok() && !odom_active) { ros::Duration(0.1).sleep(); }
    
    // 2. Mappa Condivisa
    map_builder::MapBuilder builder(nh, 1000.0);
    ROS_INFO("Building Map...");
    Map map = builder.buildMap(); 

    std::vector<BenchmarkResult> results;

    // --- LOOP BENCHMARK ---
    for (const std::string& planner_type : PLANNERS_TO_TEST) {
        
        // CHECK PRELIMINARE: Se siamo lontani dallo start (es. primo run fallito), torna indietro
        driveBackToStart(dubins_client, map, nullptr); // nullptr = genera PRM temporanea
        clearMap(debug_pub); // Pulisce RViz SOLO ORA, quando siamo pronti

        ROS_WARN("\n=== STARTING BENCHMARK: %s ===", planner_type.c_str());
        
        BenchmarkResult res;
        res.method = planner_type;
        res.dubins_retries = 0;
        res.victims_collected = 0;
        res.success = false;

        std::shared_ptr<Roadmap> roadmap = nullptr;

        try {
            // A. ROADMAP
            auto t1 = std::chrono::high_resolution_clock::now();
            roadmap = generateRoadmap(planner_type, map);
            
            integratePosition(roadmap, INITIAL_START_POSE, map.obstacles.get_obstacles(), "Start");
            Vertex gatePose(0,0);
            if (!map.gates.get_gates().empty()) {
                Point g = map.gates.get_gates()[0].get_position();
                gatePose = Vertex(g.x, g.y);
                integratePosition(roadmap, gatePose, map.obstacles.get_obstacles(), "Gate");
            }
            std::vector<Victim> victims = map.victims.get_victims();
            for(size_t i=0; i<victims.size(); ++i) {
                Point v = victims[i].get_center();
                integratePosition(roadmap, Vertex(v.x, v.y), map.obstacles.get_obstacles(), "Victim " + std::to_string(i));
            }
            res.roadmap_time_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t1).count();

            // B. PLAN
            auto t3 = std::chrono::high_resolution_clock::now();
            std::vector<int> missionSequence = GraphSearch::TaskPlanner::planMissionSequence(*roadmap, INITIAL_START_POSE, victims, gatePose);
            
            std::set<int> criticalNodes(missionSequence.begin(), missionSequence.end());
            std::vector<int> fullGlobalPath;
            for (size_t i = 0; i < missionSequence.size() - 1; ++i) {
                std::vector<int> rawSegment = GraphSearch::AStarPlanner::computePath(*roadmap, missionSequence[i], missionSequence[i+1]);
                if (rawSegment.empty()) continue; 
                bool isCriticalApproach = (i == missionSequence.size() - 2);
                std::vector<int> segmentToAdd = isCriticalApproach ? rawSegment : optimizePath(rawSegment, *roadmap, map.obstacles.get_obstacles());
                if (fullGlobalPath.empty()) fullGlobalPath = segmentToAdd;
                else fullGlobalPath.insert(fullGlobalPath.end(), segmentToAdd.begin() + 1, segmentToAdd.end());
            }
            res.planning_time_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t3).count();

            if (fullGlobalPath.empty()) throw std::runtime_error("Path Empty.");
            GraphSearch::rviz_plan(fullGlobalPath, *roadmap, debug_pub);

            // C. EXECUTION
            ROS_INFO("Executing Mission (%lu nodes)...", fullGlobalPath.size());
            auto t5 = std::chrono::high_resolution_clock::now();

            for (size_t i = 0; i < fullGlobalPath.size() - 1; ++i) {
                int nextIdx = fullGlobalPath[i+1];
                const Vertex& currentV = roadmap->getVertex(fullGlobalPath[i]);
                const Vertex& targetV = roadmap->getVertex(nextIdx);
                bool isCritical = (criticalNodes.find(nextIdx) != criticalNodes.end());
                
                double goal_theta = std::atan2(targetV.y - currentV.y, targetV.x - currentV.x);
                // (Logica angoli semplificata per brevità, usare tua logica completa se preferisci)

                int retry_count = 0;
                bool node_success = false;
                while (!node_success && retry_count <= (isCritical ? 3 : 1)) {
                    dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, ROBOT_VELOCITY, TURNING_RADIUS);
                    
                    ros::Time start_t = ros::Time::now();
                    ros::Time stall_t = ros::Time::now();
                    while(ros::ok()) {
                        if (dubins_client.waitForResult(0.02)) { if(dubins_client.isSuccess()) node_success = true; break; }
                        double v; { std::lock_guard<std::mutex> lock(odom_mutex); v = current_speed; }
                        if (v > 0.05) stall_t = ros::Time::now();
                        if (ros::Time::now() - stall_t > ros::Duration(3.0)) break; // Stalled
                    }
                    if(node_success) break;
                    retry_count++; res.dubins_retries++;
                }

                if (!node_success && isCritical) break;
                if (isCritical) { 
                    res.victims_collected++; 
                    dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, 0.0, TURNING_RADIUS);
                    ros::Duration(0.5).sleep(); 
                }
            }
            res.execution_time_sec = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t5).count();
            if (res.victims_collected > 0) res.success = true;

        } catch (const std::exception& e) {
            ROS_ERROR("Planner Failed: %s", e.what());
            res.notes = e.what();
        }

        results.push_back(res);
        
        // FINE MISSIONE: Il robot tornerà indietro all'inizio del prossimo ciclo (o ora)
        // Lo chiamiamo esplicitamente qui per sicurezza prima di stampare il log
        ROS_INFO("Mission Ended. Returning to Start...");
        driveBackToStart(dubins_client, map, roadmap); // Usa la roadmap corrente per tornare!
    }

    // REPORT
    std::cout << "\n\n=== FINAL REPORT ===\n";
    printf("%-10s | %-8s | %-8s | %-7s | %s\n", "Method", "Map(ms)", "Exec(s)", "Vict", "Notes");
    for (const auto& r : results) printf("%-10s | %-8.0f | %-8.2f | %-7d | %s\n", r.method.c_str(), r.roadmap_time_ms, r.execution_time_sec, r.victims_collected, r.notes.c_str());
    std::cout << "====================\n";

    return 0;
}