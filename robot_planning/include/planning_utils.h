#ifndef PLANNING_UTILS_H
#define PLANNING_UTILS_H

#include <vector>
#include <cmath>

#include <map_library/map_data_structures.h> 
#include <libraries/roadmap/roadmap_data_structures.h>

namespace PlanningUtils {
    // Converts a Point to a Vertex
    Vertex toVertex(const Point& p);

    // Checks id a point lies inside a polygon
    bool pointInPolygon(const Vertex& point, const std::vector<Vertex>& polygon); 

    // Checks if a point lies inside a specific obstacle
    bool pointInObstacle(const Vertex& point, const Obstacle& obstacle);

    // Checks if a point lies inside any obstacle in the list
    bool pointInAnyObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles);

    // Checks if a line segment intersects with any obstacle
    bool lineSegmentIntersectsObstacle(const Vertex& p1, const Vertex& p2, 
                                       const std::vector<Obstacle>& obstacles);

    // Calculates the intersection point of two line segments (AB and CD)
    bool getSegmentIntersection(const Vertex& A, const Vertex& B, 
                                const Vertex& C, const Vertex& D, 
                                Vertex& intersection);

    // Calculates the Euclidean distance to the nearest obstacle boundary
    double distanceToNearestObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles);
    
    // It checks if the segment between p1 and p2 is free from obstacles with a minimum clearance
    bool isSegmentSafe(const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles, double min_clearance);

    // Checks if a vertex is already in a list (with epsilon tolerance)
    bool containsVertex(const std::vector<Vertex>& list, const Vertex& p, double epsilon = 1e-5);

    // Check if a point is navigable (inside map borders, outside obstacles)
    bool isPointValid(double x, double y, const Map& map, double min_clearance = 0.0);
    
    // Integrate a position into the roadmap, connecting it to nearby nodes if possible
    void integratePosition( std::shared_ptr<Roadmap>& roadmap, const Vertex& pos, const std::vector<Obstacle>& obstacles, const std::string& label);
    
    // Optimize a path
    std::vector<int> optimizePath(const std::vector<int>& rawPath, const Roadmap& roadmap, const std::vector<Obstacle>& obstacles, double SAFETY_MARGIN=0.5);
}

#endif // PLANNING_UTILS_H