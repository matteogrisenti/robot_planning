#include <limits>
#include <algorithm>
#include <cmath>

#include "combinatorial_planning/planning_utils.h"

namespace PlanningUtils {

    bool pointInPolygon(const Vertex& point, const std::vector<Vertex>& polygon) {
        bool inside = false;
        size_t n = polygon.size();
        
        // Loop through every edge of the polygon
        // j is the previous vertex, i is the current vertex
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const Vertex& v1 = polygon[i];
            const Vertex& v2 = polygon[j];

            // Check if the ray from 'point' intersects the edge (v1, v2)
            // 1. (v1.y > point.y) != (v2.y > point.y): 
            //    Ensures the point's Y coordinate is within the Y-range of the edge.
            //    One vertex must be above, and the other below the point's Y.
            
            // 2. point.x < ... :
            //    Calculates the X-coordinate of the intersection between the ray (y = point.y)
            //    and the edge (v1-v2). If point.x is to the left of this intersection,
            //    the ray to the right definitely hits the edge.
            
            bool intersects = ((v1.y > point.y) != (v2.y > point.y)) &&
                              (point.x < (v2.x - v1.x) * (point.y - v1.y) / (v2.y - v1.y) + v1.x);

            if (intersects) {
                inside = !inside;
            }
        }

        return inside;
    }

    bool pointInObstacle(const Vertex& point, const Obstacle& obstacle) {
        std::vector<Point> polygon = obstacle.get_points();
        int n = polygon.size();
        bool inside = false;
        
        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = polygon[i].x, yi = polygon[i].y;
            double xj = polygon[j].x, yj = polygon[j].y;
            
            bool intersect = ((yi > point.y) != (yj > point.y)) &&
                            (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        
        return inside;
    }

    bool pointInAnyObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles) {
        for (const auto& obstacle : obstacles) {
            if (pointInObstacle(point, obstacle)) {
                return true;
            }
        }
        return false;
    }

    bool lineSegmentIntersectsObstacle(
        const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles) {
        
        // Simple collision check: sample points along the line
        int numSamples = 20;
        for (int i = 0; i <= numSamples; i++) {
            double t = static_cast<double>(i) / numSamples;
            Vertex sample(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y));
            
            if (pointInAnyObstacle(sample, obstacles)) {
                return true;
            }
        }
        return false;
    }

    double distanceToNearestObstacle(
        const Vertex& point, const std::vector<Obstacle>& obstacles) {
        
        double minDist = std::numeric_limits<double>::max();
        
        for (const auto& obstacle : obstacles) {
            for (const auto& obstacle_point : obstacle.get_points()) {
                Vertex vertex = Vertex(point.x, point.y);
                double dist = point.distance(vertex);
                minDist = std::min(minDist, dist);
            }
        }
        
        return minDist;
    }

}