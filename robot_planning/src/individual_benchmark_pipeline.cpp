#include <ros/ros.h>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <set> 
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <nav_msgs/Odometry.h> 
#include <algorithm> 
#include <mutex>
#include <chrono>

#include "map_library.h"
#include "roadmap.h"
#include "roadmap_factory.h"
#include "astar.h" 
#include "dubins_planner_client.h"
#include "planning_utils.h" 

// =============================================================================
// 1. CONFIGURAZIONE HYPERPARAMETERS (Stile Individual Benchmark)
// =============================================================================
const double ROBOT_VELOCITY = 0.5;       
const double TURNING_RADIUS = 0.4;
const int MAX_DUBINS_RETRIES = 3;           // Max retries per nodi critici
const double SAFETY_MARGIN = 0.90;          // Distanza minima ostacoli (High Res Check)
const std::string OUTPUT_DIR = "src/robot_planning/robot_planning/src/test/";
const std::string METRICS_FILENAME = "benchmark_results.txt";

// =============================================================================
// 2. GLOBALS & STRUCTS
// =============================================================================
std::mutex odom_mutex;
double current_speed = 0.0;
Vertex current_pose_odom(0,0); 
bool odom_active = false;

struct RunMetrics {
    std::string planner;
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

// Salva metriche su file (Esteso con dati di missione)
void appendMetricsToFile(const std::string& filepath, const RunMetrics& m) {
    std::ofstream file;
    file.open(filepath, std::ios_base::app); 
    
    if (file.is_open()) {
        file.seekp(0, std::ios::end);
        if (file.tellp() == 0) {
            file << "Planner\tRoadmap_T(s)\tPlanning_T(s)\tExec_T(s)\tVictims\tRetries\tSuccess\n";
        }
        
        file << m.planner << "\t" 
             << m.t_roadmap << "\t" 
             << m.t_total_planning << "\t"
             << m.t_execution << "\t"
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

// Controllo collisioni ad alta risoluzione (da complete_pipeline)
bool isSegmentSafe(const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles, double min_clearance) {
    if (PlanningUtils::lineSegmentIntersectsObstacle(p1, p2, obstacles)) return false;
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    double step_size = 0.05; 
    int steps = std::max(2, (int)(dist / step_size)); 
    for (int i = 0; i <= steps; ++i) {
        double t = (double)i / steps;
        Vertex p(p1.x + t*(p2.x - p1.x), p1.y + t*(p2.y - p1.y));
        if (PlanningUtils::distanceToNearestObstacle(p, obstacles) < min_clearance) {
            return false; 
        }
    }
    return true;
}

// Integrazione robusta con retry su margini diversi (da complete_pipeline)
void integratePosition(std::shared_ptr<Roadmap>& roadmap, const Vertex& pos, const std::vector<Obstacle>& obstacles, const std::string& label) {
    // Evita duplicati se il punto è già vicinissimo a un nodo esistente
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

    // Prova a connettere con margini di sicurezza decrescenti se necessario
    std::vector<double> margins = {SAFETY_MARGIN, 0.60, 0.30}; 
    int connected_count = 0;
    int min_connections = 3; 

    for (double margin : margins) {
        if (connected_count >= min_connections) break;
        for(const auto& pair : neighbors) {
            if (connected_count >= 15) break; 
            int targetIdx = pair.second;
            
            // Check se arco esiste già
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
    if(connected_count > 0) ROS_INFO("Integrated %s at (%.2f, %.2f) -> %d edges.", label.c_str(), pos.x, pos.y, connected_count);
    else ROS_ERROR("CRITICAL: FAILED to integrate %s!", label.c_str());
}

// Ottimizzatore percorso (da complete_pipeline)
std::vector<int> optimizePath(const std::vector<int>& rawPath, const Roadmap& roadmap, const std::vector<Obstacle>& obstacles) {
    if (rawPath.size() < 2) return rawPath;
    std::vector<int> optimized;
    optimized.push_back(rawPath[0]);
    int currentIdx = 0;
    double min_node_dist = 1.0; 

    while (currentIdx < rawPath.size() - 1) {
        bool shortcutFound = false;
        for (int i = rawPath.size() - 1; i > currentIdx + 1; --i) {
            const Vertex& vStart = roadmap.getVertex(rawPath[currentIdx]);
            const Vertex& vEnd = roadmap.getVertex(rawPath[i]);
            if (isSegmentSafe(vStart, vEnd, obstacles, SAFETY_MARGIN)) {
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
    
    // Filtering nodi troppo vicini
    if (optimized.size() > 2) {
        std::vector<int> filtered;
        filtered.push_back(optimized[0]);
        for (size_t i = 1; i < optimized.size() - 1; ++i) {
            const Vertex& prev = roadmap.getVertex(filtered.back());
            const Vertex& curr = roadmap.getVertex(optimized[i]);
            if (std::hypot(curr.x - prev.x, curr.y - prev.y) > min_node_dist) {
                filtered.push_back(optimized[i]);
            }
        }
        filtered.push_back(optimized.back());
        return filtered;
    }
    return optimized;
}

// =============================================================================
// 4. MAIN
// =============================================================================
int main(int argc, char **argv) {
    ros::init(argc, argv, "unified_benchmark_node");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    // Setup Odom
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);
    if (odom_sub.getNumPublishers() == 0) {
        odom_sub = nh.subscribe("/limo0/odom", 1, odomCallback);
    }

    // Parametro Planner Type (da ROS param)
    std::string planner_type;
    nh.param<std::string>("planner_type", planner_type, "acd"); // Default "acd"
    
    ros::Publisher debug_pub = nh.advertise<visualization_msgs::Marker>("/debug_path", 10); 
    DubinsClient dubins_client("/dubins_planner_server/follow_dubins_path");

    ROS_WARN("=== BENCHMARK STARTED ===");
    ROS_INFO("Planner: %s | Vel: %.1f | Safety: %.2f", planner_type.c_str(), ROBOT_VELOCITY, SAFETY_MARGIN);

    // Attesa Odometria
    ROS_INFO("Waiting for Odometry...");
    while(ros::ok() && !odom_active) ros::Duration(0.1).sleep();
    
    Vertex startPose;
    {
        std::lock_guard<std::mutex> lock(odom_mutex);
        startPose = current_pose_odom;
    }
    ROS_INFO("Start Pose Locked: (%.2f, %.2f)", startPose.x, startPose.y);

    RunMetrics metrics;
    metrics.planner = planner_type;
    metrics.victims_collected = 0;
    metrics.dubins_retries = 0;
    metrics.success = false;

    try {
        // --- BUILD MAP ---
        map_builder::MapBuilder builder(nh, 1000.0);
        ROS_INFO("Building Map...");
        Map map = builder.buildMap();

        // --- PHASE 1: ROADMAP GENERATION ---
        ros::Time t_start_roadmap = ros::Time::now();
        std::shared_ptr<Roadmap> roadmap = generateRoadmap(planner_type, map);
        if (!roadmap) throw std::runtime_error("Roadmap generation failed.");
        
        // Integrazione Entità
        integratePosition(roadmap, startPose, map.obstacles.get_obstacles(), "Start");
        
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
        
        metrics.t_roadmap = (ros::Time::now() - t_start_roadmap).toSec();
        roadmap->plot(false, true, OUTPUT_DIR + planner_type + "_roadmap.png");

        // --- PHASE 2: PLANNING ---
        ros::Time t_start_plan = ros::Time::now();
        
        ROS_INFO("Planning Mission Sequence...");
        std::vector<int> missionSequence = GraphSearch::TaskPlanner::planMissionSequence(*roadmap, startPose, victims, gatePose);
        if (missionSequence.empty()) throw std::runtime_error("Task Planning failed (Sequence empty).");

        std::set<int> criticalNodes(missionSequence.begin(), missionSequence.end());
        std::vector<int> fullGlobalPath;

        for (size_t i = 0; i < missionSequence.size() - 1; ++i) {
            std::vector<int> rawSegment = GraphSearch::AStarPlanner::computePath(*roadmap, missionSequence[i], missionSequence[i+1]);
            
            if (rawSegment.empty()) {
                ROS_ERROR("Path failed between nodes %d -> %d. Skipping segment.", missionSequence[i], missionSequence[i+1]);
                continue; 
            }

            bool isCriticalApproach = (i == missionSequence.size() - 2);
            std::vector<int> segmentToAdd;
            
            // Non ottimizzare l'approccio finale al gate per evitare tagli pericolosi
            if (isCriticalApproach) {
                segmentToAdd = rawSegment;
            } else {
                segmentToAdd = optimizePath(rawSegment, *roadmap, map.obstacles.get_obstacles());
            }

            if (fullGlobalPath.empty()) fullGlobalPath = segmentToAdd;
            else fullGlobalPath.insert(fullGlobalPath.end(), segmentToAdd.begin() + 1, segmentToAdd.end());
        }

        metrics.t_total_planning = (ros::Time::now() - t_start_plan).toSec();

        if (fullGlobalPath.empty()) throw std::runtime_error("Final Global Path is empty.");
        
        // Debug Visuals
        roadmap_viz::RoadmapVisualizer viz;
        viz.render(map, *roadmap);
        viz.drawPath(*roadmap, fullGlobalPath);
        viz.saveToFile(OUTPUT_DIR + planner_type + "_path.png");
        GraphSearch::rviz_plan(fullGlobalPath, *roadmap, debug_pub);

        // --- PHASE 3: EXECUTION ---
        ROS_INFO("Starting Execution...");
        ros::Time t_start_exec = ros::Time::now();
        bool mission_failed = false;

        for (size_t i = 0; i < fullGlobalPath.size() - 1; ++i) {
            int currentIdx = fullGlobalPath[i];
            int nextIdx = fullGlobalPath[i+1];
            const Vertex& currentV = roadmap->getVertex(currentIdx);
            const Vertex& targetV = roadmap->getVertex(nextIdx);

            bool isCritical = (criticalNodes.find(nextIdx) != criticalNodes.end());
            
            // Calcolo orientamento target
            double goal_theta = 0.0;
            double approach_angle = std::atan2(targetV.y - currentV.y, targetV.x - currentV.x);

            if (isCritical) {
                goal_theta = approach_angle;
            } 
            else if (i + 2 < fullGlobalPath.size()) {
                const Vertex& futureV = roadmap->getVertex(fullGlobalPath[i+2]);
                double exit_angle = std::atan2(futureV.y - targetV.y, futureV.x - targetV.x);
                double turn_angle = std::abs(normalizeAngle(exit_angle - approach_angle));
                if (turn_angle > (60.0 * M_PI / 180.0)) goal_theta = approach_angle; 
                else goal_theta = exit_angle; 
            } else {
                goal_theta = approach_angle;
            }

            // Check se siamo già arrivati (Safety per nodi critici)
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

            // Loop di movimento (con Retries)
            int retry_count = 0;
            int current_max_retries = isCritical ? MAX_DUBINS_RETRIES : 1;
            bool node_success = already_at_target;

            while (!node_success && retry_count <= current_max_retries) {
                // Riduce velocità nei retry
                double v_cmd = (retry_count == 0) ? ROBOT_VELOCITY : (ROBOT_VELOCITY * 0.6); 
                
                dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, v_cmd, TURNING_RADIUS);
                
                // Monitoraggio stallo
                ros::Rate loop_rate(5); 
                ros::Time stall_start_time = ros::Time::now();
                ros::Time segment_start_time = ros::Time::now(); 

                while(ros::ok()) {
                    if (dubins_client.waitForResult(0.01)) { 
                        if (dubins_client.isSuccess()) node_success = true;
                        break;
                    }

                    // Anti-Stall Logic
                    double v_now = 0.0;
                    { std::lock_guard<std::mutex> lock(odom_mutex); v_now = current_speed; }

                    if (ros::Time::now() - segment_start_time > ros::Duration(2.0)) { // Grace period iniziale
                        if (v_now > 0.02) stall_start_time = ros::Time::now();
                        else if (ros::Time::now() - stall_start_time > ros::Duration(3.0)) {
                            ROS_WARN("STALL DETECTED moving to node %d", nextIdx);
                            break; // Esce dal wait loop, triggera retry o fail
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

            // Esito nodo
            if (!node_success) {
                if (isCritical) {
                    ROS_ERROR("CRITICAL FAILURE at Node %d. Mission Aborted.", nextIdx);
                    mission_failed = true;
                    break; 
                } else {
                    ROS_WARN("Failed non-critical node %d. Skipping...", nextIdx);
                }
            } else if (isCritical && !already_at_target) {
                metrics.victims_collected++; // Nota: incrementa anche per il Gate, aggiustare se necessario logica task
                ROS_INFO("CRITICAL TARGET REACHED (Node %d). Victims: %d", nextIdx, metrics.victims_collected);
                dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, 0.0, TURNING_RADIUS);
                ros::Duration(1.0).sleep();
            }
        }

        metrics.t_execution = (ros::Time::now() - t_start_exec).toSec();
        metrics.success = !mission_failed;

        // Logging finale
        ROS_INFO("\n=== MISSION REPORT ===");
        ROS_INFO("Planner: %s", metrics.planner.c_str());
        ROS_INFO("Success: %s", metrics.success ? "YES" : "NO");
        ROS_INFO("Time Plan: %.3fs | Time Exec: %.3fs", metrics.t_total_planning, metrics.t_execution);
        ROS_INFO("Victims/Gate Points: %d | Retries: %d", metrics.victims_collected, metrics.dubins_retries);

        appendMetricsToFile(OUTPUT_DIR + METRICS_FILENAME, metrics);

    } catch (const std::exception& e) {
        ROS_ERROR("CRITICAL EXCEPTION: %s", e.what());
        return 1;
    }

    return 0;
}