#ifndef APPROXIMATE_CELL_DECOMPOSITION_H
#define APPROXIMATE_CELL_DECOMPOSITION_H

#include <vector>
#include <memory>
#include <map/map_data_structures.h>
#include <roadmap/roadmap_data_structures.h>

namespace ApproximateDecomposition {

    // Main Function: Recursively subdivides space into grid cells
    std::shared_ptr<Roadmap> approximateCellDecomposition(
        const Map& map, 
        int maxDepth = 4,
        double minCellSize = 0.5
    );

    // --- Helper Functions ---

    // Recursive function to process and split cells
    // Returns void, populates the 'freeCells' vector
    void recursiveDecomposition(const Cell& currentCell, const Map& map, 
                                int depth, int maxDepth, double minCellSize, 
                                std::vector<Cell>& freeCells);

    // Return a vertex for each cell ( check if the center is a valid: inside the map) 
    Vertex calculateRefinedCentroid(const Cell& cell, const Map& map);

    // Checks if a cell intersects *any* obstacle (Modified for box collision)
    bool cellIntersectsObstacle(const Cell& cell, const Map& map);
    bool getSegmentIntersection(const Vertex& A, const Vertex& B, 
                              const Vertex& C, const Vertex& D, 
                              Vertex& intersection);
    Vertex toVertex(const Point& p);
    
    // Connects adjacent free cells to form the graph
    void connectAdjacentCells(const std::vector<Cell>& cells, std::shared_ptr<Roadmap> roadmap);

}

#endif // APPROXIMATE_CELL_DECOMPOSITION_H