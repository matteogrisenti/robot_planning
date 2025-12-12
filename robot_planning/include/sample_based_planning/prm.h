#ifndef PRM_H
#define PRM_H

#include <memory>
#include <map/map_data_structures.h>
#include <roadmap/roadmap_data_structures.h>

namespace sample_planning {

    struct PRMConfig {
        int num_samples = 500;      // N nella slide
        int k_neighbors = 10;       // K nella slide
        double max_connection_dist = -1.0; // Opzionale: limita la lunghezza massima degli edge
    };

    /**
     * @brief Builds a Probabilistic Roadmap (PRM)
     * * Implements the standard algorithm:
     * 1. Sampling Phase: Uniform random sampling in C_free
     * 2. Connection Phase: Connects each node to K-nearest neighbors if collision-free
     */
    std::shared_ptr<Roadmap> buildPRM(const Map& map, const PRMConfig& config);

}

#endif // PRM_H