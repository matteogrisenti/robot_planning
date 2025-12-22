#include <ros/ros.h>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <set> 
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <nav_msgs/Odometry.h> 
#include <algorithm> 

#include "map_library.h"
#include "roadmap.h"
#include "roadmap_factory.h"
#include "astar.h" 
#include "dubins_planner_client.h"
#include "planning_utils.h" 

// Globals
std::mutex odom_mutex;
double current_speed = 0.0;
Vertex current_pose_odom(0,0); 
bool odom_active = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(odom_mutex);
    current_speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    current_pose_odom.x = msg->pose.pose.position.x;
    current_pose_odom.y = msg->pose.pose.position.y;
    odom_active = true;
}

// --- SAFETY CHECK (HIGH RESOLUTION) ---
bool isSegmentSafe(const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles, double min_clearance) {
    // 1. Line Check Veloce
    if (PlanningUtils::lineSegmentIntersectsObstacle(p1, p2, obstacles)) return false;

    // 2. Check Approfondito ad Alta Risoluzione
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    
    // FIX: Passo di campionamento ridotto a 5cm per non mancare spigoli
    double step_size = 0.05; 
    int steps = std::max(2, (int)(dist / step_size)); 
    
    for (int i = 0; i <= steps; ++i) {
        double t = (double)i / steps;
        Vertex p(p1.x + t*(p2.x - p1.x), p1.y + t*(p2.y - p1.y));
        
        // Se la distanza minima è inferiore al clearance richiesto, è collisione
        if (PlanningUtils::distanceToNearestObstacle(p, obstacles) < min_clearance) {
            return false; 
        }
    }
    return true;
}

// --- ROADMAP INTEGRATION (ADAPTIVE) ---
// Qui dobbiamo essere flessibili: se la vittima è attaccata al muro, dobbiamo accettare un margine basso.
void integratePosition(std::shared_ptr<Roadmap>& roadmap, const Vertex& pos, const std::vector<Obstacle>& obstacles, const std::string& label) {
    for(int i=0; i<roadmap->getNumVertices(); ++i) {
        if(roadmap->getVertex(i).distance(pos) < 0.05) return; 
    }

    int newIdx = roadmap->addVertex(pos);
    
    // Cerchiamo connessioni su un raggio ampio
    double search_radius = 15.0; 
    std::vector<std::pair<double, int>> neighbors;
    for(int i=0; i<roadmap->getNumVertices(); ++i) {
        if(i == newIdx) continue;
        double d = roadmap->getVertex(i).distance(pos);
        if(d < search_radius) neighbors.push_back({d, i});
    }
    std::sort(neighbors.begin(), neighbors.end());

    // STRATEGIA ADATTIVA:
    // 1. Proviamo a connetterci con un margine ENORME (0.9m) per garantire curve Dubins facili.
    // 2. Se fallisce, scendiamo a margini più stretti (0.45m, 0.20m) solo per i punti difficili.
    std::vector<double> margins = {0.90, 0.60, 0.30, -1.0}; 
    int connected_count = 0;
    int min_connections = 3; 

    for (double margin : margins) {
        if (connected_count >= min_connections) break;

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

    if(connected_count > 0) {
        ROS_INFO("Integrated %s at (%.2f, %.2f) -> %d edges.", label.c_str(), pos.x, pos.y, connected_count);
    } else {
        ROS_ERROR("CRITICAL: FAILED to integrate %s! It is unreachable.", label.c_str());
    }
}

// --- OPTIMIZER (STRICT SAFETY) ---
std::vector<int> optimizePath(const std::vector<int>& rawPath, const Roadmap& roadmap, const std::vector<Obstacle>& obstacles) {
    if (rawPath.size() < 2) return rawPath;
    std::vector<int> optimized;
    optimized.push_back(rawPath[0]);
    int currentIdx = 0;
    
    // FIX: Margine di sicurezza AUMENTATO a 0.90m.
    // Questo impedisce all'ottimizzatore di creare scorciatoie che passano 
    // a meno di 90cm dagli ostacoli. Se il percorso originale (roadmap) è sicuro,
    // l'ottimizzatore lo manterrà invece di tagliare pericolosamente.
    double safety_margin = 0.90; 
    
    // Distanza minima aumentata per evitare micro-segmenti che mandano in crisi Dubins
    double min_node_dist = 1.0; 

    while (currentIdx < rawPath.size() - 1) {
        bool shortcutFound = false;
        // Cerca il nodo più lontano visibile in SICUREZZA
        for (int i = rawPath.size() - 1; i > currentIdx + 1; --i) {
            const Vertex& vStart = roadmap.getVertex(rawPath[currentIdx]);
            const Vertex& vEnd = roadmap.getVertex(rawPath[i]);
            
            // Check severo
            if (isSegmentSafe(vStart, vEnd, obstacles, safety_margin)) {
                // Check lunghezza segmento (evita segmenti troppo lunghi in spazi stretti se necessario, ma qui ok)
                optimized.push_back(rawPath[i]);
                currentIdx = i;
                shortcutFound = true;
                break;
            }
        }
        // Se non trovo scorciatoie sicure, avanzo di un solo passo sul percorso originale
        // (che si presume sicuro grazie alla costruzione della Roadmap/Integrazione)
        if (!shortcutFound) {
            optimized.push_back(rawPath[currentIdx + 1]);
            currentIdx++;
        }
    }
    
    // Filtro finale per rimuovere nodi troppo vicini
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

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "complete_planning_pipeline_node");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);
    if (odom_sub.getNumPublishers() == 0) {
        odom_sub = nh.subscribe("/limo0/odom", 1, odomCallback);
    }

    std::string planner_type;
    nh.param<std::string>("planner_type", planner_type, "rrt"); 
    std::string general_output_dir = "src/robot_planning/robot_planning/src/test/";
    double robot_velocity = 0.5;   
    double turning_radius = 0.4;   
    ros::Publisher debug_pub = nh.advertise<visualization_msgs::Marker>("/debug_path", 10); 
    
    ROS_INFO("Waiting for initial robot pose from Odom...");
    while(ros::ok() && !odom_active) {
        ros::Duration(0.1).sleep();
    }
    Vertex startPose;
    {
        std::lock_guard<std::mutex> lock(odom_mutex);
        startPose = current_pose_odom;
    }
    ROS_INFO("Planning from Real Pose: (%.2f, %.2f)", startPose.x, startPose.y);

    try {
        map_builder::MapBuilder builder(nh, 1000.0);
        ROS_INFO("Waiting for Map Data...");
        Map map = builder.buildMap();
        
        ROS_INFO("Generating Roadmap: %s", planner_type.c_str());
        std::shared_ptr<Roadmap> roadmap = generateRoadmap(planner_type, map);
        if (!roadmap) throw std::runtime_error("Failed to generate roadmap.");
        
        integratePosition(roadmap, startPose, map.obstacles.get_obstacles(), "RealStart");

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

        roadmap->plot(false, true, general_output_dir + planner_type + "_roadmap.png");

        ROS_INFO("Planning Mission Sequence...");
        std::vector<int> missionSequence = GraphSearch::TaskPlanner::planMissionSequence(*roadmap, startPose, victims, gatePose);
        if (missionSequence.empty()) throw std::runtime_error("Task Planning failed.");

        std::set<int> criticalNodes(missionSequence.begin(), missionSequence.end());

        std::vector<int> fullGlobalPath;
        for (size_t i = 0; i < missionSequence.size() - 1; ++i) {
            std::vector<int> rawSegment = GraphSearch::AStarPlanner::computePath(*roadmap, missionSequence[i], missionSequence[i+1]);
            
            if (rawSegment.empty()) {
                ROS_ERROR("PATH FAILED segment %lu. Skipping.", i);
                continue; 
            }

            // Ottimizzazione con margini SEVERI
            std::vector<int> optimizedSegment = optimizePath(rawSegment, *roadmap, map.obstacles.get_obstacles());
            if (fullGlobalPath.empty()) fullGlobalPath = optimizedSegment;
            else fullGlobalPath.insert(fullGlobalPath.end(), optimizedSegment.begin() + 1, optimizedSegment.end());
        }

        if (fullGlobalPath.empty()) throw std::runtime_error("Failed to compute path.");
        roadmap_viz::RoadmapVisualizer viz;
        viz.render(map, *roadmap);
        viz.drawPath(*roadmap, fullGlobalPath);
        viz.saveToFile(general_output_dir + planner_type + "_path.png");
        GraphSearch::rviz_plan(fullGlobalPath, *roadmap, debug_pub);

        ROS_INFO("Initializing Dubins Action Client...");
        DubinsClient dubins_client("/dubins_planner_server/follow_dubins_path");

        ROS_INFO("Starting Execution...");
        for (size_t i = 0; i < fullGlobalPath.size() - 1; ++i) {
            int currentIdx = fullGlobalPath[i];
            int nextIdx = fullGlobalPath[i+1];
            const Vertex& currentV = roadmap->getVertex(currentIdx);
            const Vertex& targetV = roadmap->getVertex(nextIdx);

            bool isCritical = (criticalNodes.find(nextIdx) != criticalNodes.end());
            
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

            int retry_count = 0;
            const int MAX_RETRIES = isCritical ? 5 : 0; 
            bool success = false;

            while (!success && retry_count <= MAX_RETRIES) {
                double v_cmd = (retry_count == 0) ? robot_velocity : (robot_velocity * 0.6); 
                dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, v_cmd, turning_radius);
                
                ros::Rate loop_rate(5); 
                ros::Time stall_start_time = ros::Time::now();
                ros::Time segment_start_time = ros::Time::now(); 

                while(ros::ok()) {
                    if (dubins_client.waitForResult(0.01)) { 
                        if (dubins_client.isSuccess()) success = true;
                        else ROS_WARN("Node %d aborted by server.", nextIdx);
                        break;
                    }

                    double v_now = 0.0;
                    { std::lock_guard<std::mutex> lock(odom_mutex); v_now = current_speed; }

                    if (ros::Time::now() - segment_start_time < ros::Duration(2.0)) {
                        stall_start_time = ros::Time::now(); 
                    } else {
                        if (v_now > 0.02) stall_start_time = ros::Time::now();
                        else if (ros::Time::now() - stall_start_time > ros::Duration(3.0)) {
                            ROS_ERROR("STALL DETECTED. Aborting.");
                            break;
                        }
                    }
                    loop_rate.sleep();
                }

                if (success) break;
                
                retry_count++;
                if (retry_count <= MAX_RETRIES) {
                    ROS_WARN("Retrying Critical Node %d (Attempt %d)...", nextIdx, retry_count);
                    ros::Duration(0.5).sleep();
                }
            }

            if (!success) {
                if (isCritical) {
                    ROS_ERROR("CRITICAL FAILURE: Could not reach VICTIM/GATE (Node %d).", nextIdx);
                } else {
                    ROS_WARN("Skipping intermediate waypoint %d.", nextIdx);
                }
            } 
            else if (isCritical) {
                ROS_INFO(">>> CRITICAL TARGET REACHED (Node %d).", nextIdx);
                dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, 0.0, turning_radius);
                ros::Duration(1.0).sleep();
            }
        }
        ROS_INFO("Mission Completed.");
    } catch (const std::exception& e) {
        ROS_ERROR("CRITICAL ERROR: %s", e.what());
        return 1;
    }
    return 0;
}