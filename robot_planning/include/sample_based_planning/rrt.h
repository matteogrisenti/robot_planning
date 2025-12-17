#ifndef RRT_H
#define RRT_H

#include <memory>
#include <map/map_data_structures.h>
#include <roadmap/roadmap_data_structures.h>

namespace sample_planning {

    struct RRTConfig {
        int max_iterations = 2000;  // Sicurezza per evitare loop infiniti
        double step_size = 0.5;     // Lunghezza massima dell'estensione (EXTEND)
        double goal_bias = 0.1;     // Probabilità di campionare il goal invece di un punto random
        
        // Se true, l'algoritmo si ferma appena arriva vicino al goal.
        // Se false, esplora fino a max_iterations (utile per vedere tutto l'albero).
        bool stop_at_goal = false;  
        Vertex goal_point = Vertex(0,0); // Punto target (se stop_at_goal è true)
        double goal_tolerance = 1.0;     // Distanza per considerare il goal raggiunto
    };

    /**
     * @brief Builds an RRT (Rapidly-exploring Random Tree) following the standard algorithm.
     *
     */
    std::shared_ptr<Roadmap> buildRRT(const Map& map, const RRTConfig& config);

}

#endif // RRT_H