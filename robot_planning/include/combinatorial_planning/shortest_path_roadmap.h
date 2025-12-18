#ifndef SHORTEST_PATH_ROADMAP_H
#define SHORTEST_PATH_ROADMAP_H

#include <memory>
#include "map_library/map_data_structures.h"
#include "roadmap/roadmap_data_structures.h"

/**
 * @brief Generates a Shortest Path Roadmap (Reduced Visibility Graph).
 * @param map The input Map object.
 * @param padding Optional padding distance to inflate obstacles.
 * @return Roadmap A populated roadmap containing the visibility graph.
 */
std::shared_ptr<Roadmap> shortestPathRoadmap(const Map& map, const double padding = 0.0);


std::vector<Point> applyPaddingToPolygon(const std::vector<Point>& poly, double padding = 0.0);

#endif // SHORTEST_PATH_ROADMAP_H