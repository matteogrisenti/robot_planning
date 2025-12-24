#include <limits>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

#include "planning_utils.h"

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


    // It checks if the segment between p1 and p2 is free from obstacles with a minimum clearance
    bool isSegmentSafe(const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles, double min_clearance) {
        if (PlanningUtils::lineSegmentIntersectsObstacle(p1, p2, obstacles)) return false;
        
        double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
        int steps = std::max(2, (int)(dist / 0.05));

        // Check points along the segment for minimum clearance
        for (int i = 0; i <= steps; ++i) {
            double t = (double)i / steps;
            Vertex p(p1.x + t*(p2.x - p1.x), p1.y + t*(p2.y - p1.y));
            if (PlanningUtils::distanceToNearestObstacle(p, obstacles) < min_clearance)
                return false;
        }
        return true;
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

    // Check if a point is navigable (inside map borders, outside obstacles)
    bool isPointValid(double x, double y, const Map& map) {
        std::vector<Vertex> mapPoly;
        for(const auto& p : map.borders.get_points())
            mapPoly.push_back(PlanningUtils::toVertex(p));

        // 1. Must be INSIDE the map borders
        if(PlanningUtils::pointInPolygon( Vertex(x, y), mapPoly) == false) {
            return false; 
        }

        // 2. Must be OUTSIDE all obstacles
        return !PlanningUtils::pointInAnyObstacle(Vertex(x, y), map.obstacles.get_obstacles());
    }


    // Integrate a position into the roadmap, connecting it to nearby nodes if possible
    void integratePosition(
        std::shared_ptr<Roadmap>& roadmap, const Vertex& pos, const std::vector<Obstacle>& obstacles, const std::string& label) {
    
        // Check if position already exists
        if(PlanningUtils::containsVertex(
            std::vector<Vertex>(roadmap->getNumVertices()), pos, 0.05)) {
            return; // Already present
        }

        // Check if position is valid
        if(!PlanningUtils::isPointValid(pos.x, pos.y, *(roadmap->getMap()))) {
            return; // Invalid position
        }
        
        // Add new vertex
        int newIdx = roadmap->addVertex(pos);
        double search_radius = 15.0;                        // Search radius for neighbors
        std::vector<std::pair<double, int>> neighbors;
        
        // Find nearby vertices
        for (int i = 0; i < roadmap->getNumVertices(); ++i) {
            if (i == newIdx) continue;
            double d = roadmap->getVertex(i).distance(pos);     // Distance to new position
            if (d < search_radius) neighbors.push_back({d, i});
        }
        std::sort(neighbors.begin(), neighbors.end());  // Sort by distance
        
        // Try to connect to neighbors with varying safety margins
        std::vector<double> margins = {0.90, 0.60, 0.30};   // Safety margins in meters
        int connected_count = 0;

        for (double margin : margins) {

            // Stop if already connected enough: 3 connections
            if (connected_count >= 3) break;  
            
            // Attempt connections between new vertex and neighbors
            for (const auto& pair : neighbors) {

                // Limit total connections to 15
                if (connected_count >= 15) break;   
                
                int targetIdx = pair.second; 
                bool edgeExists = false;

                // Check if edge already exists
                for (const auto& e : roadmap->getEdges(newIdx))
                    if (e.targetVertex == targetIdx) edgeExists = true;
                if (edgeExists) continue;
                
                // Check if the segment is safe
                bool possible = false;
                if (margin > 0.0)
                    possible = PlanningUtils::isSegmentSafe(pos, roadmap->getVertex(targetIdx), obstacles, margin);
                else
                    possible = !PlanningUtils::lineSegmentIntersectsObstacle(pos, roadmap->getVertex(targetIdx), obstacles);
                
                // Add edge if safe
                if (possible) {
                    roadmap->addEdge(newIdx, targetIdx, pair.first);
                    roadmap->addEdge(targetIdx, newIdx, pair.first);
                    connected_count++;
                }
            }
        }
    }


    // Optimize path by removing unnecessary waypoints while ensuring safety margins
    std::vector<int> optimizePath(const std::vector<int>& rawPath, const Roadmap& roadmap, const std::vector<Obstacle>& obstacles, double SAFETY_MARGIN) {
        if (rawPath.size() < 2) return rawPath;
        
        std::vector<int> optimized;         // optimized path
        optimized.push_back(rawPath[0]);    // initialize the optimized path 
        int currentIdx = 0;                 
        
        // Scroll all the path, and try to find a shorcut between the current point and the 
        // final point of the path
        while (currentIdx < rawPath.size() - 1) {
            bool shortcutFound = false;
            for (int i = rawPath.size() - 1; i > currentIdx + 1; --i) {
                const Vertex& vStart = roadmap.getVertex(rawPath[currentIdx]);
                const Vertex& vEnd = roadmap.getVertex(rawPath[i]);

                if (PlanningUtils::isSegmentSafe(vStart, vEnd, obstacles, SAFETY_MARGIN)) {
                    optimized.push_back(rawPath[i]);
                    currentIdx = i;
                    shortcutFound = true;
                    ROS_INFO("[PLANNING UTILS]: Shortcut found!");
                    break;
                }
            }
            if (!shortcutFound) {
                ROS_INFO("[PLANNING UTILS]: Shortcut not found!");
                optimized.push_back(rawPath[currentIdx + 1]);
                currentIdx++;
            }
        }
        
        //  
        if (optimized.size() > 2) {
            std::vector<int> filtered;
            filtered.push_back(optimized[0]);
            for (size_t i = 1; i < optimized.size() - 1; ++i) {
                const Vertex& prev = roadmap.getVertex(filtered.back());
                const Vertex& curr = roadmap.getVertex(optimized[i]);
                if (std::hypot(curr.x - prev.x, curr.y - prev.y) > 1.0) {
                    filtered.push_back(optimized[i]);
                }
            }
            filtered.push_back(optimized.back());
            return filtered;
        }
        return optimized;
    }

}