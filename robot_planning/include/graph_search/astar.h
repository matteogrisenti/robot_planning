#ifndef GRAPH_SEARCH_H
#define GRAPH_SEARCH_H

#include <vector>
#include <map>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

#include "roadmap/roadmap_data_structures.h"
#include "map/map_data_structures.h"

namespace GraphSearch {

    // --- A* PLANNER (Low Level) ---
    class AStarPlanner {
    public:
        /**
         * @brief Calcola il percorso ottimo tra due nodi del grafo.
         * @return std::vector<int> Lista di indici dei nodi (path). Vuoto se fallisce.
         */
        static std::vector<int> computePath(const Roadmap& graph, int startNodeIdx, int goalNodeIdx);

    private:
        struct NodeWrapper {
            int id;
            double f_score;
            bool operator>(const NodeWrapper& other) const { return f_score > other.f_score; }
        };

        static double heuristic(const Vertex& a, const Vertex& b);
    };

    // --- TASK PLANNER (High Level) ---
    class TaskPlanner {
    public:
        /**
         * @brief Determina la sequenza di visita: Start -> Vittime (Greedy) -> Gate
         * * @param graph La Roadmap su cui navigare.
         * @param startPos Posizione cartesiana del robot.
         * @param victims Lista delle vittime dalla Mappa.
         * @param gatePos Posizione cartesiana del Gate (Goal finale).
         * @return std::vector<int> La sequenza ORDINATA dei nodi del grafo da visitare.
         * Es: [NodeStart, NodeVictim3, NodeVictim1, NodeGate]
         */
        static std::vector<int> planMissionSequence(
            const Roadmap& graph, 
            const Vertex& startPos, 
            const std::vector<Victim>& victims, 
            const Vertex& gatePos
        );

    private:
        // Helper per trovare il nodo del grafo più vicino a un punto fisico
        static int getNearestNodeIdx(const Roadmap& graph, const Vertex& pos);
    };

}

#endif // GRAPH_SEARCH_H