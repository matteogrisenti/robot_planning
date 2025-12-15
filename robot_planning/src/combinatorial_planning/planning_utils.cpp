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

        // 1. Edge Intersection Check
        // Iterate over all obstacles and their edges
        for (const auto& obs : obstacles) {
            const std::vector<Point>& pts = obs.get_points();
            size_t n = pts.size();
            if (n < 2) continue;

            for (size_t i = 0; i < n; ++i) {
                Vertex vC = toVertex(pts[i]);
                Vertex vD = toVertex(pts[(i + 1) % n]);

                // Check if our path (p1-p2) crosses this obstacle edge (vC-vD)
                // We ignore cases where the path shares a vertex with the obstacle
                // (p1==vC, p1==vD, etc.) because starting at an obstacle is allowed.
                
                // Euclidean distance check to ignore shared endpoints
                double d1 = std::hypot(p1.x - vC.x, p1.y - vC.y);
                double d2 = std::hypot(p1.x - vD.x, p1.y - vD.y);
                double d3 = std::hypot(p2.x - vC.x, p2.y - vC.y);
                double d4 = std::hypot(p2.x - vD.x, p2.y - vD.y);
                
                if (d1 < 1e-5 || d2 < 1e-5 || d3 < 1e-5 || d4 < 1e-5) {
                    continue; // Shared vertex, not a collision
                }

                Vertex intersection;
                if (getSegmentIntersection(p1, p2, vC, vD, intersection)) {
                    return true; // Collision detected
                }
            }
        }

        // 2. Midpoint Check
        // Necessary for cases where the segment is fully INSIDE a convex obstacle
        // (e.g., connecting two vertices of the same polygon through the inside).
        Vertex mid;
        mid.x = (p1.x + p2.x) / 2.0;
        mid.y = (p1.y + p2.y) / 2.0;

        // Use existing point check
        if (pointInAnyObstacle(mid, obstacles)) {
            return true; 
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

    // Helpers to extract Roadmap Node from Cell
    Vertex toVertex(const Point& p) { return Vertex(p.x, p.y); }

    // Calculate the intersection point of two line segments (AB and CD)
    // Returns true if they intersect, and stores point in 'intersection'
    bool getSegmentIntersection(const Vertex& A, const Vertex& B, 
                              const Vertex& C, const Vertex& D, 
                              Vertex& intersection) {
        // Line AB represented as a1x + b1y = c1
        double a1 = B.y - A.y;
        double b1 = A.x - B.x;
        double c1 = a1 * A.x + b1 * A.y;

        // Line CD represented as a2x + b2y = c2
        double a2 = D.y - C.y;
        double b2 = C.x - D.x;
        double c2 = a2 * C.x + b2 * C.y;

        double determinant = a1 * b2 - a2 * b1;

        if (std::abs(determinant) < 1e-9) {
            return false; // Parallel lines
        }

        double x = (b2 * c1 - b1 * c2) / determinant;
        double y = (a1 * c2 - a2 * c1) / determinant;
        
        // Check if intersection is strictly within both segments
        auto onSegment = [](const Vertex& p, const Vertex& start, const Vertex& end) {
            return p.x >= std::min(start.x, end.x) - 1e-7 && p.x <= std::max(start.x, end.x) + 1e-7 &&
                   p.y >= std::min(start.y, end.y) - 1e-7 && p.y <= std::max(start.y, end.y) + 1e-7;
        };

        Vertex p(x, y);
        if (onSegment(p, A, B) && onSegment(p, C, D)) {
            intersection = p;
            return true;
        }
        return false;
    }


    // Helper to check if a vertex already exists in the list (within tolerance)
    bool containsVertex(const std::vector<Vertex>& list, const Vertex& p, double epsilon) {
        for (const auto& v : list) {
            // Check Euclidean distance (squared is faster, but hypot is clearer)
            double dx = v.x - p.x;
            double dy = v.y - p.y;
            if ((dx*dx + dy*dy) < (epsilon * epsilon)) {
                return true;
            }
        }
        return false;
    }

}