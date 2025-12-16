#ifndef APPROXIMATE_CELL_DECOMPOSITION_H
#define APPROXIMATE_CELL_DECOMPOSITION_H

#include <vector>
#include <memory>
#include <map/map_data_structures.h>
#include <roadmap/roadmap_data_structures.h>

namespace ApproximateDecomposition {

    // Main Function
    std::shared_ptr<Roadmap> approximateCellDecomposition(
        const Map& map, 
        int maxDepth = 4,
        double minCellSize = 0.5
    );

    // Helpers
    void recursiveDecomposition(const Cell& currentCell, const Map& map, 
                                int depth, int maxDepth, double minCellSize, 
                                std::vector<Cell>& freeCells);

    Vertex calculateRefinedCentroid(const Cell& cell, const Map& map);

    bool cellIntersectsObstacle(const Cell& cell, const Map& map);
    
    void connectAdjacentCells(const std::vector<Cell>& cells, std::shared_ptr<Roadmap> roadmap);
}

#endif // APPROXIMATE_CELL_DECOMPOSITION_H