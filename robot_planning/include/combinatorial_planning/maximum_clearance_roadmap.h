#ifndef ROADMAP_GENERATOR_H
#define ROADMAP_GENERATOR_H

#include <memory>
#include "map/map_data_structures.h"
#include "roadmap/roadmap_data_structures.h"

/**
 * @brief Generates a Maximum Clearance Roadmap (Medial Axis) using the Voronoi Diagram.
 * * Uses the Boost.Polygon library (Line Sweep Algorithm) internally to compute 
 * the Voronoi diagram of the map borders and obstacles.
 * * @param map Pointer or reference to the loaded Map.
 * @return Roadmap A populated Roadmap structure containing the graph.
 */
std::shared_ptr<Roadmap>  generateMaxClearanceRoadmap(const Map& map);


namespace HelperMaxClearanceRoadmap {
    
    // INTERNAL HELPER STRUCTURES
    // The Boost.Polygon library requires integer coordinates
    // We scale the map coordinates to maintain precision
    const double SCALING_FACTOR = 1000.0;
    // Create a 2D integer point structure
    struct PointInt {
        int x, y;
    };
    // Create a integer segment structure using two PointInt
    struct SegmentInt {
        PointInt p0;
        PointInt p1;
    };

    // Adds polygon edges to the segment list for Voronoi computation
    void addPolygonToSegments(const std::vector<Point>& polyPoints, 
                              std::vector<SegmentInt>& segments);
}

#endif // ROADMAP_GENERATOR_H