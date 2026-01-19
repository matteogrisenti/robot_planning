#ifndef EXACT_CELL_DECOMPOSITION_H
#define EXACT_CELL_DECOMPOSITION_H

#include <vector>
#include <memory>
#include <cmath>
#include <utility>
#include <map_library/map_data_structures.h>
#include <libraries/roadmap.h>


/**  Main Function: Recursively subdivides space into grid cells
 * @brief Generates a Roadmap using Exact Cell Decomposition 
 * @param map: The Map to decompose
 * @return Shared pointer to the generated Roadmap
 */
std::shared_ptr<Roadmap> exactCellDecomposition(const Map& map);


namespace HelperExactDecomposition {
    
    std::vector<Trapezoid> computeTrapezoidalDecomposition(const Map& map);
    
    void connectAdjacentTrapezoids(std::vector<Trapezoid>& trapezoids);
}

#endif // EXACT_CELL_DECOMPOSITION_H