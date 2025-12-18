#include <ros/ros.h>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <visualization_msgs/Marker.h>

#include "map_library.h"
#include "roadmap.h"
#include "roadmap_factory.h"
#include "astar.h" 
#include "dubins_planner_client.h"

/** Complete Planning Pipeline
 *  1. Map Building
 *  2. Roadmap Generation (Combinatorial or Sample-Based)
 *  3. Task Planning + A* Pathfinding
 *  4. Execution on Robot: Iterative loop on each step of the path
 *     4.1. Build a Dubins Trajectory 
 *     4.2. Execute the trajectory on the Robot
 */

int main(int argc, char **argv) {
    ros::init(argc, argv, "complete_planning_pipeline_node");
    ros::NodeHandle nh("~");

    std::string planner_type = "mcr"; // "prm", "rrt", "rrt_star", "ecd", "acd", "mcr", "spr"

    std::string general_output_dir = "src/robot_planning/src/test/";

    double robot_velocity = 0.3;   // m/s
    double turning_radius = 0.6;   // meters

    // RVIZ PUBLISHER FOR DEBUGGING
    ros::Publisher debug_pub = nh.advertise<visualization_msgs::Marker>("/debug_path", 10); 
    // Wait for RVIZ to subscribe
    ros::Duration(1.0).sleep();

    try {
        // =================
        // 1. Map Building
        // =================
        map_builder::MapBuilder builder(nh, 1000.0);
        ROS_INFO("Waiting for Map Data...");
        Map map = builder.buildMap();
        ROS_INFO("Map built successfully.");


        // =========================
        // 2. Generte the Roadmap 
        // =========================
        //    - Combinatorial Planning: ECD, ACD, MCR, SPR
        //    - Sampling Based Planning: PRM, RTT, RTT*)
        std::shared_ptr<Roadmap> roadmap = generateRoadmap(planner_type, map);
        if (!roadmap) {
            throw std::runtime_error("Failed to generate roadmap.");
        }
        ROS_INFO("Roadmap generated with %d vertices.", roadmap->getNumVertices());
        // Optional: Save Roadmap Image
        std::string roadmap_image_path = general_output_dir + planner_type + "_roadmap.png";
        roadmap->plot(false, true, roadmap_image_path);
        ROS_INFO("Roadmap image saved to %s", roadmap_image_path.c_str());


        // ===================================
        // 3. Task Planning + A* Pathfinding
        // ===================================
        GraphSearch::AStarPlanner planner;
        GraphSearch::TaskPlanner task_planner;

        // Task Planning
        // Prepara coordinate Start e Gate
        Point s = map.start.get_position();
        Vertex startPose(s.x, s.y);
        Vertex gatePose(0,0);
        if (!map.gates.get_gates().empty()) {
            Point g = map.gates.get_gates()[0].get_position();
            gatePose = Vertex(g.x, g.y);
        }
        std::vector<int> missionSequence = GraphSearch::TaskPlanner::planMissionSequence(*roadmap, startPose, map.victims.get_victims(), gatePose);
        if (missionSequence.empty()) {
            throw std::runtime_error("Task Planning failed: no mission sequence generated.");
        }

        // Pathfinding with A*
        std::vector<int> fullGlobalPath;
        bool pathSuccess = true;
        if (missionSequence.empty()) pathSuccess = false;

        for (size_t i = 0; i < missionSequence.size() - 1; ++i) {
            std::vector<int> segment = GraphSearch::AStarPlanner::computePath(*roadmap, missionSequence[i], missionSequence[i+1]);
            if (segment.empty()) {
                ROS_WARN("Segment %lu unreachable (%d -> %d)", i, missionSequence[i], missionSequence[i+1]);
                pathSuccess = false; 
                break;
            }
            fullGlobalPath.insert(fullGlobalPath.end(), segment.begin(), segment.end());
        }
        if (pathSuccess && !fullGlobalPath.empty()) {
            ROS_INFO("Full path computed successfully. Path length: %lu", fullGlobalPath.size());

            // Optional: Save Path Image
            roadmap_viz::RoadmapVisualizer viz;
            viz.render(map, *roadmap);
            viz.drawPath(*roadmap, fullGlobalPath);
            std::string path_image_path = general_output_dir + planner_type + "_path.png";
            if (viz.saveToFile(path_image_path)) {
                ROS_INFO("Path image saved to %s", path_image_path.c_str());
            } else {
                ROS_ERROR("Failed to save path image to %s", path_image_path.c_str());
            }
            // RVIZ Visualization
            GraphSearch::rviz_plan(fullGlobalPath, *roadmap, debug_pub);
        } else {
            ROS_ERROR("Failed to compute full path.");
        }


        // ===================================
        // 4. Execution on Robot 
        // ===================================
        ROS_INFO("Initializing Dubins Action Client...");
        
        // Connect to the server named "follow_dubins_path" defined in dubins_planner_server.cpp
        DubinsClient dubins_client("/dubins_planner_server/follow_dubins_path");

        ROS_INFO("Starting Execution Phase...");
        // Iterate through the path points
        // We start from index 1 because the robot is presumably already at index 0 (Start)
        for (size_t i = 0; i < fullGlobalPath.size() - 1; ++i) {
            
            int currentIdx = fullGlobalPath[i];
            int nextIdx = fullGlobalPath[i+1];

            const Vertex& targetVertex = roadmap->getVertex(nextIdx);

            // --- CALCULATE GOAL THETA ---
            // To ensure smooth motion, the goal orientation at the target node 
            // should point towards the subsequent node in the path.
            double goal_theta = 0.0;

            if (i + 2 < fullGlobalPath.size()) {
                // If there is a node after the next one, aim towards it
                int futureIdx = fullGlobalPath[i+2];
                const Vertex& futureVertex = roadmap->getVertex(futureIdx);
                goal_theta = std::atan2(futureVertex.y - targetVertex.y, 
                                        futureVertex.x - targetVertex.x);
            } else {
                // If nextIdx is the final destination, maintain the approach angle
                const Vertex& currentVertex = roadmap->getVertex(currentIdx);
                goal_theta = std::atan2(targetVertex.y - currentVertex.y, 
                                        targetVertex.x - currentVertex.x);
            }

            // 4.1 Send Goal to Server
            dubins_client.sendGoal(
                targetVertex.x, 
                targetVertex.y, 
                goal_theta, 
                robot_velocity, 
                turning_radius
            );

            // 4.2 Wait for completion
            bool finished_before_timeout = dubins_client.waitForResult(30.0); // 30s timeout per segment

            if (!finished_before_timeout) {
                ROS_ERROR("Action timed out moving to node %d", nextIdx);
                break;
            }

            if (!dubins_client.isSuccess()) {
                ROS_ERROR("Dubins Planner failed to reach node %d", nextIdx);
                break;
            }
            
            ROS_INFO("Reached Node %d / %lu", (int)i+1, fullGlobalPath.size()-1);
            // Pause 5 seconds between goals for
            ros::Duration(5.0).sleep();
        }

        ROS_INFO("Mission Completed.");


    } catch (const std::exception& e) {
        ROS_ERROR("CRITICAL ERROR: %s", e.what());
        return 1;
    }

    return 0;
}