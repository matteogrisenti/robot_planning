#ifndef SHORTEST_PATH_ROADMAP_H
#define SHORTEST_PATH_ROADMAP_H

#include "map/map_data_structures.h"
#include "roadmap/roadmap_data_structures.h"

/**
 * @brief Generates a Shortest Path Roadmap (Reduced Visibility Graph).
 * * LOGIC:
 * 1. Vertices: Includes all "Reflex" vertices.
 * - For Obstacles (Convex): All vertices are reflex relative to C-free.
 * - For Map Borders: Only concave corners (internal angle > 180 deg) are reflex.
 * 2. Edges: 
 * - Existing edges of the obstacles/borders (if navigable).
 * - "Bitangent" lines: Connections between mutually visible reflex vertices 
 * that do not intersect the interior of any obstacle.
 * * @param map The input Map object.
 * @return Roadmap A populated roadmap containing the visibility graph.
 */
Roadmap generateShortestPathRoadmap(const Map& map);

#endif // SHORTEST_PATH_ROADMAP_H