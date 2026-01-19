#include "astar.h"
#include <iostream>
#include <ros/ros.h> // Per logging

namespace GraphSearch {

    // --- IMPLEMENTAZIONE A* BASE ---
    double AStarPlanner::heuristic(const Vertex& a, const Vertex& b) {
        // Distanza Euclidea come euristica
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    std::vector<int> AStarPlanner::computePath(const Roadmap& graph, int startNodeIdx, int goalNodeIdx) {
        if (startNodeIdx < 0 || goalNodeIdx < 0) return {};
        if (startNodeIdx == goalNodeIdx) return {startNodeIdx};

        std::priority_queue<NodeWrapper, std::vector<NodeWrapper>, std::greater<NodeWrapper>> open_set;
        std::map<int, double> g_score;
        std::map<int, int> came_from;

        const Vertex& goalVertex = graph.getVertex(goalNodeIdx);
        
        g_score[startNodeIdx] = 0.0;
        open_set.push({startNodeIdx, heuristic(graph.getVertex(startNodeIdx), goalVertex)});

        while (!open_set.empty()) {
            int u = open_set.top().id;
            open_set.pop();

            if (u == goalNodeIdx) {
                // Ricostruzione percorso
                std::vector<int> path;
                while (came_from.find(u) != came_from.end()) {
                    path.push_back(u);
                    u = came_from[u];
                }
                path.push_back(startNodeIdx);
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (const auto& edge : graph.getEdges(u)) {
                int v = edge.targetVertex;
                double tentative_g = g_score[u] + edge.weight;

                if (g_score.find(v) == g_score.end() || tentative_g < g_score[v]) {
                    came_from[v] = u;
                    g_score[v] = tentative_g;
                    double f = tentative_g + heuristic(graph.getVertex(v), goalVertex);
                    open_set.push({v, f});
                }
            }
        }
        return {}; // Nessun percorso trovato
    }

    // --- IMPLEMENTAZIONE TASK PLANNER ---
    
    int TaskPlanner::getNearestNodeIdx(const Roadmap& graph, const Vertex& pos) {
        int bestIdx = -1;
        double minDist = std::numeric_limits<double>::max();
        for (int i = 0; i < graph.getNumVertices(); ++i) {
            double d = graph.getVertex(i).distance(pos);
            if (d < minDist) { minDist = d; bestIdx = i; }
        }
        return bestIdx;
    }

    // Calcola la distanza effettiva sul grafo (usando A*) per stime precise dei tempi
    double TaskPlanner::getGraphDistance(const Roadmap& graph, int startIdx, int endIdx) {
        if (startIdx == endIdx) return 0.0;
        
        std::vector<int> path = AStarPlanner::computePath(graph, startIdx, endIdx);
        
        if (path.empty()) return 1e9; // Infinito (irraggiungibile)
        
        double dist = 0.0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            dist += graph.getVertex(path[i]).distance(graph.getVertex(path[i+1]));
        }
        return dist;
    }

    std::vector<int> TaskPlanner::planMissionSequence(const Roadmap& graph, const Vertex& startPos, 
                                                      const std::vector<Victim>& victims, const Vertex& gatePos,
                                                      double time_limit, double robot_velocity) {
        std::vector<int> sequence;
        
        // 1. Map entities to Graph Nodes
        int startNode = getNearestNodeIdx(graph, startPos);
        int gateNode = getNearestNodeIdx(graph, gatePos);

        if (startNode == -1 || gateNode == -1) {
            ROS_ERROR("[TaskPlanner] Start or Gate not connected to roadmap.");
            return {};
        }

        // Struttura interna per gestire i candidati
        struct Candidate { 
            int nodeIdx; 
            int originalIdx;
            double value; 
            bool visited; 
        };
        
        std::vector<Candidate> candidates;
        for(size_t i=0; i<victims.size(); ++i) {
            Point p = victims[i].get_center(); 
            // MODIFICATO: Usa il raggio come valore/punteggio della vittima
            double val = victims[i].get_radius(); 
            candidates.push_back({getNearestNodeIdx(graph, Vertex(p.x, p.y)), (int)i, val, false});
        }

        // Inizializzazione Loop Greedy
        sequence.push_back(startNode);
        int currentNode = startNode;
        double currentTime = 0.0;
        
        ROS_INFO("[TaskPlanner] Planning with Time Limit: %.1fs, Avg Vel: %.1f m/s", time_limit, robot_velocity);

        while (true) {
            int bestIdx = -1;
            double bestScore = -1.0;
            double timeConsumedForBest = 0.0;

            // Cerca la vittima migliore tra quelle non visitate
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (!candidates[i].visited) {
                    
                    // Calcolo Distanze (Costi)
                    double distToVictim = getGraphDistance(graph, currentNode, candidates[i].nodeIdx);
                    double distToGate = getGraphDistance(graph, candidates[i].nodeIdx, gateNode);
                    
                    // Se una vittima è irraggiungibile (es. isolata), ignorala
                    if (distToVictim > 1e8 || distToGate > 1e8) continue;

                    // Robot velocity è già ridotta del 15% per sicurezza (nel benchmark)
                    double timeToVictim = distToVictim / robot_velocity;
                    double timeToExit = distToGate / robot_velocity;

                    // --- CHECK DEL BUDGET ---
                    // Possiamo andare alla vittima E poi scappare al gate?
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

            // Se abbiamo trovato un candidato valido
            if (bestIdx != -1) {
                sequence.push_back(candidates[bestIdx].nodeIdx);
                candidates[bestIdx].visited = true;
                currentNode = candidates[bestIdx].nodeIdx;
                currentTime += timeConsumedForBest; 
                
                ROS_INFO("  -> Added Victim %d (Val: %.1f). Time used: %.1f/%.1f", 
                         candidates[bestIdx].originalIdx, candidates[bestIdx].value, currentTime, time_limit);
            } else {
                // Nessuna altra vittima è raggiungibile rientrando nel budget
                ROS_INFO("  -> No more reachable victims within budget. Heading to Gate.");
                break;
            }
        }
        
        // 3. Aggiungi sempre il Gate alla fine
        sequence.push_back(gateNode);
        
        // Debug finale
        double finalDist = getGraphDistance(graph, currentNode, gateNode);
        double totalTime = currentTime + (finalDist / robot_velocity);
        ROS_INFO("[TaskPlanner] Plan Finalized. Est. Total Time: %.1fs (Limit: %.1fs)", totalTime, time_limit);

        return sequence;
    }
}