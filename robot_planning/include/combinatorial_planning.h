#ifndef COMBINATORIAL_PLANNING_H
#define COMBINATORIAL_PLANNING_H

#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <set>

#include <map/map_data_structures.h> 
#include <roadmap/roadmap_data_structures.h>

class CombinatorialPlanning {
public:
    // Exact Cell Decomposition: Trapezoidal decomposition
    // Decomposes free space into trapezoids using vertical sweep line
    static std::shared_ptr<Roadmap> exactCellDecomposition(const Map& map);
    
    // Approximate Cell Decomposition: Quadtree-based decomposition
    // Recursively subdivides space into grid cells (free/occupied/mixed)
    static std::shared_ptr<Roadmap> approximateCellDecomposition(
        const Map& map, 
        int maxDepth = 8,
        double minCellSize = 0.1);
    
    // Maximum Clearance Roadmap: Voronoi diagram-based
    // Creates paths that maximize distance from obstacles
    static std::shared_ptr<Roadmap> maximumClearanceRoadmap(
        const Map& map,
        double gridResolution = 0.5);

private:    
    // Geometric helpers
    static bool pointInPolygon(const Vertex& point, const std::vector<Vertex>& polygon);
    static bool pointInObstacle(const Vertex& point, const Obstacle& obstacle);
    static bool pointInAnyObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles);
    static bool lineSegmentIntersectsObstacle(const Vertex& p1, const Vertex& p2, 
                                              const std::vector<Obstacle>& obstacles);
    static double distanceToNearestObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles);
    static std::pair<double, double> getMapYBoundsAtX(double x, const std::vector<Point>& boundary);
    
    // Exact cell decomposition helpers
    static std::vector<Trapezoid> computeTrapezoidalDecomposition(const Map& map);
    static void connectAdjacentTrapezoids(std::vector<Trapezoid>& trapezoids);
    
    // Approximate cell decomposition helpers
    static void subdivideCell(const Cell& cell, const Map& map, int depth, int maxDepth,
                             double minCellSize, std::vector<Cell>& freeCells);
    static bool isCellFree(const Cell& cell, const Map& map);
    static bool isCellOccupied(const Cell& cell, const Map& map);
    
    // Maximum clearance helpers
    static std::vector<std::vector<double>> computeDistanceField(const Map& map, double resolution);
    static std::vector<Vertex> extractRidgePoints(const std::vector<std::vector<double>>& distField,
                                                  const Map& map, double resolution, double threshold);
};

#endif // COMBINATORIAL_PLANNING_H