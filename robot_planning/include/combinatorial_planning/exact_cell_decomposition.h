#ifndef EXACT_CELL_DECOMPOSITION_H
#define EXACT_CELL_DECOMPOSITION_H

#include <vector>
#include <memory>
#include <cmath>
#include <utility>
#include <map/map_data_structures.h>
#include <roadmap/roadmap_data_structures.h>

namespace ExactDecomposition {
    

    // Main API
    std::shared_ptr<Roadmap> exactCellDecomposition(const Map& map);

    // Helpers exposed for testing or modularity
    std::vector<Trapezoid> computeTrapezoidalDecomposition(const Map& map);
    void connectAdjacentTrapezoids(std::vector<Trapezoid>& trapezoids);
}

#endif // EXACT_CELL_DECOMPOSITION_H