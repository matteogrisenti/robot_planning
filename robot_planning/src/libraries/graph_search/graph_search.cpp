#include "graph_search.h"
#include <iostream>
#include <ros/ros.h> // Per logging

namespace GraphSearch {

    double AStarPlanner::heuristic(const Vertex& a, const Vertex& b) {
        // Standard distance formula: sqrt((x2-x1)^2 + (y2-y1)^2)
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    std::vector<int> AStarPlanner::computePath(const Roadmap& graph, int startNodeIdx, int goalNodeIdx) {
        // 1. EDGE CASE HANDLING
        if (startNodeIdx < 0 || goalNodeIdx < 0) return {};         // Invalid indices
        if (startNodeIdx == goalNodeIdx) return {startNodeIdx};     // Start is the goal

        // 2. DATA STRUCTURES
        // Open Set: A priority queue that stores task to be explored.
        // It is ordered by the f_score (lowest cost first).
        std::priority_queue<NodeWrapper, std::vector<NodeWrapper>, std::greater<NodeWrapper>> open_set;
        std::map<int, double> g_score;
        std::map<int, int> came_from;   // Came From: A map used to store the "parent" of each node to reconstruct the final path.

        const Vertex& goalVertex = graph.getVertex(goalNodeIdx);
        
        // 3. INITIALIZATION
        g_score[startNodeIdx] = 0.0;
        open_set.push({startNodeIdx, heuristic(graph.getVertex(startNodeIdx), goalVertex)});

        // 4. MAIN SEARCH LOOP
        while (!open_set.empty()) {
            // Extract the node with the lowest f_score (highest priority)
            int u = open_set.top().id;
            open_set.pop();

            // GOAL CHECK: If we reached the target, reconstruct and return the path.
            if (u == goalNodeIdx) {
                std::vector<int> path;

                // Backtrack from goal to start using the came_from map
                while (came_from.find(u) != came_from.end()) {
                    path.push_back(u);
                    u = came_from[u];
                }
                path.push_back(startNodeIdx);               // Add the starting node
                std::reverse(path.begin(), path.end());     // Reverse to get the path from start to goal
                return path;
            }

            // 5. NEIGHBOR EXPANSION
            // Iterate through all neighbors connected to node 'u'
            for (const auto& edge : graph.getEdges(u)) {
                int v = edge.targetVertex;

                // Calculate the "tentative" g_score: the cost to reach 'v' through 'u'
                double tentative_g = g_score[u] + edge.weight;

                // If this new path to 'v' is better than any previously recorded path
                if (g_score.find(v) == g_score.end() || tentative_g < g_score[v]) {
                    // Record the best path found so far to node 'v'
                    came_from[v] = u;
                    g_score[v] = tentative_g;

                    // Calculate total estimated cost: f(n) = g(n) + h(n)
                    // g(n): Known cost from start to current node
                    // h(n): Heuristic estimate from current node to goal
                    double f = tentative_g + heuristic(graph.getVertex(v), goalVertex);

                    // Add node 'v' to the open set for future exploration
                    open_set.push({v, f});
                }
            }
        }

        // 6. TERMINATION: If the queue is empty and goal was never reached
        return {}; 
    }

    
    





    int TaskPlanner::getNearestNodeIdx(const Roadmap& graph, const Vertex& pos) {
        int bestIdx = -1;
        double minDist = std::numeric_limits<double>::max();

        for (int i = 0; i < graph.getNumVertices(); ++i) {
            // Calculate L2 distance between vertex 'i' and target position
            double d = graph.getVertex(i).distance(pos);
            if (d < minDist) { minDist = d; bestIdx = i; }
        }

        return bestIdx;
    }



    // Calcola la distanza effettiva sul grafo (usando A*) per stime precise dei tempi
    double TaskPlanner::getGraphDistance(const Roadmap& graph, int startIdx, int endIdx) {
        if (startIdx == endIdx) return 0.0;
        
        // Run the A* algorithm to find the sequence of nodes
        std::vector<int> path = AStarPlanner::computePath(graph, startIdx, endIdx);
        
        if (path.empty()) return 1e9; // Infinito (irraggiungibile)
        
        double dist = 0.0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            // Sum up the Euclidean distances between each consecutive pair of nodes in the path
            dist += graph.getVertex(path[i]).distance(graph.getVertex(path[i+1]));
        }
        
        return dist;
    }



    std::vector<int> TaskPlanner::planMissionSequence(const Roadmap& graph, const Vertex& startPos, 
                                                      const std::vector<Victim>& victims, const Vertex& gatePos,
                                                      double time_limit, double robot_velocity) {
        std::vector<int> sequence;
        
        // 1. INITIAL MAPPING
        // Convert spatial coordinates (Start and Gate) to the nearest roadmap nodes
        int startNode = getNearestNodeIdx(graph, startPos);
        int gateNode = getNearestNodeIdx(graph, gatePos);

        if (startNode == -1 || gateNode == -1) {
            ROS_ERROR("[TaskPlanner] Start or Gate not connected to roadmap.");
            return {};
        }

        // 2. CANDIDATE PREPARATION
        // Wrap victim data into a local structure for easier tracking
        struct Candidate { 
            int nodeIdx;      // Closest graph node index
            int originalIdx;  // ID for logging purposes
            double value;     // Score (victim radius)
            bool visited;     // Track if already added to sequence
        };
        
        std::vector<Candidate> candidates;
        for(size_t i=0; i<victims.size(); ++i) {
            Point p = victims[i].get_center(); 
            // Scoring logic: Larger victims (higher radius) are prioritized
            double val = victims[i].get_radius(); 
            candidates.push_back({getNearestNodeIdx(graph, Vertex(p.x, p.y)), (int)i, val, false});
        }

        // 3. GREEDY SELECTION LOOP
        sequence.push_back(startNode);
        int currentNode = startNode;
        double currentTime = 0.0;
        
        ROS_INFO("[TaskPlanner] Planning with Time Limit: %.1fs, Avg Vel: %.1f m/s", time_limit, robot_velocity);

        while (true) {
            int bestIdx = -1;
            double bestScore = -1.0;
            double timeConsumedForBest = 0.0;

            // Iterate through all unvisited candidates to find the next best target
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (!candidates[i].visited) {
                    
                    // COST CALCULATION
                    // distToVictim: Cost to reach the potential victim
                    // distToGate: Cost to escape from the victim to the exit gate
                    double distToVictim = getGraphDistance(graph, currentNode, candidates[i].nodeIdx);
                    double distToGate = getGraphDistance(graph, candidates[i].nodeIdx, gateNode);
                    
                    // Skip victims that are disconnected from the roadmap
                    if (distToVictim > 1e8 || distToGate > 1e8) continue;

                    // Robot velocity è già ridotta del 15% per sicurezza (nel benchmark)
                    double timeToVictim = distToVictim / robot_velocity;
                    double timeToExit = distToGate / robot_velocity;

                    // --- CHECK DEL BUDGET ---
                    double projectedTotalTime = currentTime + timeToVictim + timeToExit;

                    if (projectedTotalTime <= time_limit) {
                        
                        // --- EURISTICA DI SELEZIONE ---
                        double score = candidates[i].value / (timeToVictim + timeToExit); 
                        
                        if (score > bestScore) {
                            bestScore = score;
                            bestIdx = i;
                            timeConsumedForBest = timeToVictim; // Tempo per arrivare alla vittima (non per uscire)
                        }
                    }
                }
            }

            // 4. SEQUENCE UPDATE
            if (bestIdx != -1) {
                // Add the best candidate found in this iteration
                sequence.push_back(candidates[bestIdx].nodeIdx);
                candidates[bestIdx].visited = true;
                currentNode = candidates[bestIdx].nodeIdx;
                currentTime += timeConsumedForBest; 
                
                ROS_INFO("  -> Added Victim %d (Val: %.1f). Time used: %.1f/%.1f", 
                         candidates[bestIdx].originalIdx, candidates[bestIdx].value, currentTime, time_limit);
            } else {
                // If no victims can be reached while still allowing time to return to the gate, stop searching
                ROS_INFO("  -> No more reachable victims within budget. Heading to Gate.");
                break;
            }
        }
        
        // 5. MISSION FINALIZATION
        // The gate is always the final destination of the sequence
        sequence.push_back(gateNode);
        
        // Final check for debug logs
        double finalDist = getGraphDistance(graph, currentNode, gateNode);
        double totalTime = currentTime + (finalDist / robot_velocity);
        ROS_INFO("[TaskPlanner] Plan Finalized. Est. Total Time: %.1fs (Limit: %.1fs)", totalTime, time_limit);

        return sequence;
    }
}