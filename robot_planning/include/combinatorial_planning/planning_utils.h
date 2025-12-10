#ifndef PLANNING_UTILS_H
#define PLANNING_UTILS_H

#include <vector>
#include <cmath>

#include <map/map_data_structures.h> 
#include <roadmap/roadmap_data_structures.h>

namespace PlanningUtils {
    // Checks id a point lies inside a polygon
    bool pointInPolygon(const Vertex& point, const std::vector<Vertex>& polygon); 

    // Checks if a point lies inside a specific obstacle
    bool pointInObstacle(const Vertex& point, const Obstacle& obstacle);

    // Checks if a point lies inside any obstacle in the list
    bool pointInAnyObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles);

    // Checks if a line segment intersects with any obstacle
    bool lineSegmentIntersectsObstacle(const Vertex& p1, const Vertex& p2, 
                                       const std::vector<Obstacle>& obstacles);

    // Calculates the Euclidean distance to the nearest obstacle boundary
    double distanceToNearestObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles);

}

#endif // PLANNING_UTILS_H