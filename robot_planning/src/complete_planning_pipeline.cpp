#include <ros/ros.h>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <cmath>

#include "map_library.h"
#include "roadmap.h"
#include "roadmap_factory.h"
#include "astar.h" 
#include "dubins_planner_client.h"
#include "planning_utils.h" 

/** Complete Planning Pipeline
 * 1. Map Building
 * 2. Roadmap Generation
 * 3. Task Planning + A* + PATH SMOOTHING
 * 4. Execution with MOVEMENT WATCHDOG
 */

// --- FUNZIONE DI SICUREZZA ---
bool isSegmentSafe(const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles, double min_clearance) {
    // 1. Check rapido intersezione
    if (PlanningUtils::lineSegmentIntersectsObstacle(p1, p2, obstacles)) return false;

    // 2. Check approfondito (Campionamento)
    double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
    int steps = std::max(2, (int)(dist / 0.25)); // Un controllo ogni 25cm
    
    for (int i = 0; i <= steps; ++i) {
        double t = (double)i / steps;
        Vertex p(p1.x + t*(p2.x - p1.x), p1.y + t*(p2.y - p1.y));
        if (PlanningUtils::distanceToNearestObstacle(p, obstacles) < min_clearance) {
            return false; 
        }
    }
    return true;
}

// --- FUNZIONE DI OTTIMIZZAZIONE PERCORSO ---
std::vector<int> optimizePath(const std::vector<int>& rawPath, const Roadmap& roadmap, const std::vector<Obstacle>& obstacles) {
    if (rawPath.size() < 3) return rawPath;

    std::vector<int> optimized;
    optimized.push_back(rawPath[0]);

    int currentIdx = 0;
    // Margine di sicurezza
    double safety_margin = 0.35; 

    while (currentIdx < rawPath.size() - 1) {
        bool shortcutFound = false;
        for (int i = rawPath.size() - 1; i > currentIdx + 1; --i) {
            const Vertex& v1 = roadmap.getVertex(rawPath[currentIdx]);
            const Vertex& v2 = roadmap.getVertex(rawPath[i]);

            if (isSegmentSafe(v1, v2, obstacles, safety_margin)) {
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
    return optimized;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "complete_planning_pipeline_node");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Parametri
    std::string planner_type;
    nh.param<std::string>("planner_type", planner_type, "mcr"); 

    std::string general_output_dir = "src/robot_planning/robot_planning/src/test/";

    double robot_velocity = 0.5;   
    double turning_radius = 0.4;   

    ros::Publisher debug_pub = nh.advertise<visualization_msgs::Marker>("/debug_path", 10); 
    ros::Duration(1.0).sleep(); 

    try {
        // 1. Map Building
        map_builder::MapBuilder builder(nh, 1000.0);
        ROS_INFO("Waiting for Map Data...");
        Map map = builder.buildMap();
        ROS_INFO("Map built successfully.");

        // 2. Roadmap Generation
        ROS_INFO("Generating Roadmap using: %s", planner_type.c_str());
        std::shared_ptr<Roadmap> roadmap = generateRoadmap(planner_type, map);
        if (!roadmap) throw std::runtime_error("Failed to generate roadmap.");
        roadmap->plot(false, true, general_output_dir + planner_type + "_roadmap.png");

        // 3. Task Planning
        ROS_INFO("Planning Mission Sequence...");
        Point s = map.start.get_position();
        Vertex startPose(s.x, s.y);
        Vertex gatePose(0,0);
        if (!map.gates.get_gates().empty()) {
            gatePose = Vertex(map.gates.get_gates()[0].get_position().x, map.gates.get_gates()[0].get_position().y);
        }

        std::vector<int> missionSequence = GraphSearch::TaskPlanner::planMissionSequence(*roadmap, startPose, map.victims.get_victims(), gatePose);
        if (missionSequence.empty()) throw std::runtime_error("Task Planning failed.");

        // 4. Pathfinding + Optimization
        std::vector<int> fullGlobalPath;
        for (size_t i = 0; i < missionSequence.size() - 1; ++i) {
            std::vector<int> rawSegment = GraphSearch::AStarPlanner::computePath(*roadmap, missionSequence[i], missionSequence[i+1]);
            if (rawSegment.empty()) {
                ROS_ERROR("Segment unreachable! Skipping."); 
                continue;
            }

            std::vector<int> optimizedSegment = optimizePath(rawSegment, *roadmap, map.obstacles.get_obstacles());
            ROS_INFO("Segment %lu: %lu -> %lu nodes", i, rawSegment.size(), optimizedSegment.size());

            if (fullGlobalPath.empty()) {
                fullGlobalPath = optimizedSegment;
            } else {
                fullGlobalPath.insert(fullGlobalPath.end(), optimizedSegment.begin() + 1, optimizedSegment.end());
            }
        }

        if (fullGlobalPath.empty()) throw std::runtime_error("Failed to compute path.");
        ROS_INFO("FULL PATH LENGTH: %lu nodes", fullGlobalPath.size());

        // Viz
        roadmap_viz::RoadmapVisualizer viz;
        viz.render(map, *roadmap);
        viz.drawPath(*roadmap, fullGlobalPath);
        viz.saveToFile(general_output_dir + planner_type + "_path.png");
        GraphSearch::rviz_plan(fullGlobalPath, *roadmap, debug_pub);

        // 5. Execution
        ROS_INFO("Initializing Dubins Action Client...");
        DubinsClient dubins_client("/dubins_planner_server/follow_dubins_path");

        ROS_INFO("Starting Execution...");
        
        for (size_t i = 0; i < fullGlobalPath.size() - 1; ++i) {
            int currentIdx = fullGlobalPath[i];
            int nextIdx = fullGlobalPath[i+1];

            const Vertex& currentV = roadmap->getVertex(currentIdx);
            const Vertex& targetV = roadmap->getVertex(nextIdx);

            // Orientamento verso il target (tangente al percorso)
            double goal_theta = std::atan2(targetV.y - currentV.y, targetV.x - currentV.x);

            dubins_client.sendGoal(targetV.x, targetV.y, goal_theta, robot_velocity, turning_radius);

            // --- WATCHDOG DI MOVIMENTO ---
            ros::Rate loop_rate(5); // 5 Hz
            
            ros::Time last_progress_time = ros::Time::now();
            bool segment_complete = false;

            while(ros::ok()) {
                // FIX: waitForResult nel tuo wrapper prende double (secondi), non ros::Duration
                if (dubins_client.waitForResult(0.01)) { 
                    if (dubins_client.isSuccess()) {
                        segment_complete = true;
                    } else {
                        ROS_WARN("Dubins action aborted by server.");
                    }
                    break;
                }

                // Timeout dinamico basato sulla distanza stimata
                double estimated_dist = std::hypot(targetV.x - currentV.x, targetV.y - currentV.y);
                double timeout_sec = (estimated_dist / robot_velocity) * 3.0 + 5.0; // Tempo generoso
                
                if (ros::Time::now() - last_progress_time > ros::Duration(timeout_sec)) {
                     ROS_ERROR("WATCHDOG: Segment timeout! Robot likely stuck or finished prematurely. Skipping.");
                     // FIX: Rimossa chiamata a ac_.cancelGoal() perché privata.
                     // Inviando il prossimo goal al giro successivo del for loop, 
                     // il server prelazionerà automaticamente quello vecchio.
                     break; 
                }
                
                loop_rate.sleep();
            }
        }

        ROS_INFO("Mission Completed.");

    } catch (const std::exception& e) {
        ROS_ERROR("CRITICAL ERROR: %s", e.what());
        return 1;
    }

    return 0;
}