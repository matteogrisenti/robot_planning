#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <map>
#include <queue>
#include <limits>
#include <ros/ros.h> 
#include "roadmap.h"
#include "map_library.h"

namespace GraphSearch {

    // Struttura di supporto per A*
    struct NodeWrapper {
        int id;
        double f_score;
        // Min-heap priority logic
        bool operator>(const NodeWrapper& other) const { return f_score > other.f_score; }
    };

    class AStarPlanner {
    public:
        static double heuristic(const Vertex& a, const Vertex& b);
        static std::vector<int> computePath(const Roadmap& graph, int startNodeIdx, int goalNodeIdx);
    };

    class TaskPlanner {
    public:
        // Trova il nodo più vicino a una coordinata (per ingresso/uscita dal grafo)
        static int getNearestNodeIdx(const Roadmap& graph, const Vertex& pos);

        // PIANIFICAZIONE MISSIONE CON BUDGET (Target Rescue)
        // Seleziona un sottoinsieme di vittime massimizzando il valore entro il time_limit.
        static std::vector<int> planMissionSequence(
            const Roadmap& graph, 
            const Vertex& startPos, 
            const std::vector<Victim>& victims, 
            const Vertex& gatePos,
            double time_limit,       // NUOVO: Tempo massimo totale (s)
            double robot_velocity    // NUOVO: Velocità media stimata (m/s)
        );

        // Helper per calcolare la distanza di percorrenza reale sul grafo
        static double getGraphDistance(const Roadmap& graph, int startIdx, int endIdx);
    };

    // Funzione di visualizzazione (definita in rviz_plot_plan.cpp)
    void rviz_plan(const std::vector<int>& path, const Roadmap& graph, const ros::Publisher& pub);
}