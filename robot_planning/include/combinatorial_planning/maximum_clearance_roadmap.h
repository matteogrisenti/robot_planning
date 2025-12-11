#ifndef ROADMAP_GENERATOR_H
#define ROADMAP_GENERATOR_H

#include "map/map_data_structures.h"
#include "roadmap/roadmap_data_structures.h"

/**
 * @brief Generates a Maximum Clearance Roadmap (Medial Axis) using the Voronoi Diagram.
 * * Uses the Boost.Polygon library (Line Sweep Algorithm) internally to compute 
 * the Voronoi diagram of the map borders and obstacles.
 * * @param map Pointer or reference to the loaded Map.
 * @return Roadmap A populated Roadmap structure containing the graph.
 */
Roadmap generateMaxClearanceRoadmap(const Map& map);

#endif // ROADMAP_GENERATOR_H