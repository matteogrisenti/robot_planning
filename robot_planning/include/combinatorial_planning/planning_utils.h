#ifndef PLANNING_UTILS_H
#define PLANNING_UTILS_H

#include <vector>
#include <cmath>
#include <memory>

#include <map/map_data_structures.h> 
#include <roadmap/roadmap_data_structures.h>

namespace PlanningUtils {
    // Geometry Helpers
    bool pointInPolygon(const Vertex& point, const std::vector<Vertex>& polygon); 
    bool pointInObstacle(const Vertex& point, const Obstacle& obstacle);
    bool pointInAnyObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles);
    bool lineSegmentIntersectsObstacle(const Vertex& p1, const Vertex& p2, 
                                       const std::vector<Obstacle>& obstacles);
    double distanceToNearestObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles);

    // Helpers per SPR
    bool isPointValid(double x, double y, const Map& map);
    Vertex toVertex(const Point& p);

    // Legacy
    void connectTargetsToRoadmap(std::shared_ptr<Roadmap> roadmap, const Map& map);
}

#endif // PLANNING_UTILS_H