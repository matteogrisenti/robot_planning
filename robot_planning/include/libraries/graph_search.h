#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <queue>
#include <limits>
#include <ros/ros.h> 
#include "libraries/roadmap.h"
#include "map_library.h"


namespace GraphSearch {

    struct NodeWrapper {
        int id;
        double f_score;
        bool operator>(const NodeWrapper& other) const { return f_score > other.f_score; }
    };

    


    class AStar {
    public:

        /**
         * @brief Calculates the heuristic (estimated cost) between two vertices.
         * * In A*, the heuristic must be "admissible" (it never overestimates the 
         * actual cost to reach the goal) to guarantee the shortest path.
         * This implementation uses the Euclidean Distance (Straight-line distance).
         */
        static double heuristic(const Vertex& a, const Vertex& b);


        /**
         * @brief Computes the shortest path on a roadmap graph using the A* algorithm.
         * * This function finds the optimal path from a start node to a goal node 
         * by exploring nodes based on their estimated total cost (g + h).
         * * @param graph         The Roadmap graph containing vertices and edges.
         * @param startNodeIdx  The index of the starting vertex in the graph.
         * @param goalNodeIdx   The index of the goal vertex in the graph.
         * * @return A vector of node indices representing the path; empty if no path is found.
         */
        static std::vector<int> computePath(const Roadmap& graph, int startNodeIdx, int goalNodeIdx);
    };




    class TaskPlanner {
    public:
        /**
         * @brief Spatial search to map a coordinate to the roadmap.
         * * @param graph The Roadmap containing the navigation graph.
         * @param pos The (x, y) coordinates of the target entity.
         * @return int The index of the nearest vertex; -1 if the graph is empty.
         */
        static int getNearestNodeIdx(const Roadmap& graph, const Vertex& pos);


        
        /**
         * @brief Generates a mission sequence to maximize victim rescue within a time limit.
         * * This uses a Greedy approach with a "Safety Return" check. At every step, it 
         * evaluates if visiting the next most profitable victim still allows enough 
         * time to reach the exit gate.
         * * @param graph The roadmap for navigation.
         * @param startPos Initial robot coordinates.
         * @param victims List of detected victims (with positions and radii).
         * @param gatePos Coordinates of the mission exit point.
         * @param time_limit Maximum mission duration in seconds.
         * @param robot_velocity Average speed of the robot in m/s.
         * @return std::vector<int> Ordered list of graph node indices to visit.
         */
        static std::vector<int> planMissionSequence(
            const Roadmap& graph, 
            const Vertex& startPos, 
            const std::vector<Victim>& victims, 
            const Vertex& gatePos,
            double time_limit,       
            double robot_velocity    
        );



        /**
         * @brief Calculates total path distance along graph edges.
         * * Useful for time estimation where Euclidean distance would ignore obstacles.
         * * @param graph The navigation Roadmap.
         * @param startIdx Graph index of origin.
         * @param endIdx Graph index of destination.
         * @return double The accumulated edge weights along the path; 1e9 if unreachable.
         */
        static double getGraphDistance(const Roadmap& graph, int startIdx, int endIdx);
    };


    class GraphPlanner {
    public:
        /**
         * @brief Expands a high-level mission sequence into a detailed global path.
         * * Iterates through the mission sequence, computes A* paths between segments,
         * and applies path optimization (shortcut smoothing) where appropriate.
         * * @param graph The roadmap.
         * @param missionSequence The list of high-level nodes to visit (from TaskPlanner).
         * @param obstacles The list of obstacles (required for optimization checks).
         * @return std::vector<int> The complete ordered list of nodes for the robot to follow.
         */
        static std::vector<int> computeFullTrajectory(
            const Roadmap& graph, 
            const std::vector<int>& missionSequence,
            const std::vector<Obstacle>& obstacles
        );
    };

    void rviz_plan(const std::vector<int>& path, const Roadmap& graph, const ros::Publisher& pub);
}