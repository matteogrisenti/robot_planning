#include "astar.h"

namespace GraphSearch {

    // --- IMPLEMENTAZIONE A* ---
    double AStarPlanner::heuristic(const Vertex& a, const Vertex& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    std::vector<int> AStarPlanner::computePath(const Roadmap& graph, int startNodeIdx, int goalNodeIdx) {
        if (startNodeIdx < 0 || goalNodeIdx < 0) return {};

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
        return {};
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

    std::vector<int> TaskPlanner::planMissionSequence(const Roadmap& graph, const Vertex& startPos, 
                                                      const std::vector<Victim>& victims, const Vertex& gatePos) {
        std::vector<int> sequence;
        
        // Mappatura Start e Gate
        int startNode = getNearestNodeIdx(graph, startPos);
        int gateNode = getNearestNodeIdx(graph, gatePos);

        if (startNode == -1 || gateNode == -1) return {};

        struct PendingVictim { int nodeIdx; bool visited; };
        std::vector<PendingVictim> pendingVictims;
        
        // --- FIX: Conversione esplicita da Point a Vertex ---
        for(const auto& v : victims) {
            Point p = v.get_center(); 
            // Costruiamo Vertex(x,y) manualmente
            pendingVictims.push_back({getNearestNodeIdx(graph, Vertex(p.x, p.y)), false});
        }
        // ---------------------------------------------------

        sequence.push_back(startNode);
        int currentNode = startNode;
        size_t visitedCount = 0; // size_t per evitare warning

        while (visitedCount < pendingVictims.size()) {
            int bestIdx = -1;
            double shortestDist = std::numeric_limits<double>::max();

            for (size_t i = 0; i < pendingVictims.size(); ++i) {
                if (!pendingVictims[i].visited) {
                    double d = graph.getVertex(currentNode).distance(graph.getVertex(pendingVictims[i].nodeIdx));
                    if (d < shortestDist) { shortestDist = d; bestIdx = i; }
                }
            }

            if (bestIdx != -1) {
                sequence.push_back(pendingVictims[bestIdx].nodeIdx);
                pendingVictims[bestIdx].visited = true;
                visitedCount++;
                currentNode = pendingVictims[bestIdx].nodeIdx;
            } else break;
        }
        sequence.push_back(gateNode);
        return sequence;
    }
}