#include <limits>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

#include "planning_utils.h"

namespace PlanningUtils {


    bool pointInPolygon(const Vertex& point, const std::vector<Vertex>& polygon) {
        // 'inside' tracks the parity of intersections. 
        // It starts at false (even count = 0).
        bool inside = false;
        size_t n = polygon.size();

        // Loop through each edge of the polygon.
        // i is the current vertex, j is the previous vertex to form the edge (v1, v2).
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const Vertex& v1 = polygon[i];
            const Vertex& v2 = polygon[j];

            bool intersects = ((v1.y > point.y) != (v2.y > point.y)) &&
                              (point.x < (v2.x - v1.x) * (point.y - v1.y) / (v2.y - v1.y) + v1.x);
            
            // Toggle the 'inside' state for every valid intersection found.
            if (intersects) inside = !inside;
        }
        return inside;
    }



    bool pointInObstacle(const Vertex& point, const Obstacle& obstacle) {
        // Extract the vertices of the polygon
        std::vector<Point> polygon = obstacle.get_points();
        int n = polygon.size();

        // The 'inside' variable tracks the parity of intersections (Even/Odd)
        bool inside = false;

        // Iterate through every edge of the polygon.
        // i is the current vertex, j is the previous vertex (closing the loop with n-1).
        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = polygon[i].x, yi = polygon[i].y;
            double xj = polygon[j].x, yj = polygon[j].y;

            bool intersect = ((yi > point.y) != (yj > point.y)) &&
                            (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi);

            // Every time an intersection is detected, toggle the boolean.
            // False -> True (1 intersection: Inside)
            // True -> False (2 intersections: Outside)
            if (intersect) inside = !inside;
        }
        return inside;
    }



    bool pointInAnyObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles) {
        // Calls the specific geometric containment test (usually a Ray-Casting algorithm)
        // for each individual polygon.
        for (const auto& obstacle : obstacles) {
            if (pointInObstacle(point, obstacle)) {
                return true; // Collision found; no need to check remaining obstacles.
            }
        }
        return false;
    }



    Vertex toVertex(const Point& p) { 
        return Vertex(p.x, p.y); 
    }



    bool getSegmentIntersection(const Vertex& A, const Vertex& B, 
                                const Vertex& C, const Vertex& D, 
                                Vertex& intersection) {

        // --- Phase 1: Line Equation Coefficients ---
        // General Form: ax + by = c
        // For a line through (x1, y1) and (x2, y2):
        // a = y2 - y1, b = x1 - x2, c = ax1 + by1

        // Segment 1 (AB)
        double a1 = B.y - A.y;
        double b1 = A.x - B.x;
        double c1 = a1 * A.x + b1 * A.y;

        // Segment 2 (CD)
        double a2 = D.y - C.y;
        double b2 = C.x - D.x;
        double c2 = a2 * C.x + b2 * C.y;

        // --- Phase 2: Solving for the Intersection ---
        // Determinant (D) of the coefficient matrix:
        // | a1  b1 |
        // | a2  b2 |
        double determinant = a1 * b2 - a2 * b1;
        
        // If determinant is 0 (or near zero), the lines are parallel or collinear.
        // They do not have a unique intersection point.
        if (std::abs(determinant) < 1e-9) return false; 
        
        // Using Cramer's Rule to find the intersection (x, y) of the infinite lines:
        double x = (b2 * c1 - b1 * c2) / determinant;
        double y = (a1 * c2 - a2 * c1) / determinant;
        
        // --- Phase 3: Boundary Validation ---
        // A point (x, y) can be the intersection of the infinite lines, but it must
        // lie within the bounding boxes of both finite segments to be a segment intersection.
        auto onSegment = [](const Vertex& p, const Vertex& start, const Vertex& end) {
            return p.x >= std::min(start.x, end.x) - 1e-7 && p.x <= std::max(start.x, end.x) + 1e-7 &&
                   p.y >= std::min(start.y, end.y) - 1e-7 && p.y <= std::max(start.y, end.y) + 1e-7;
        };

        Vertex p(x, y);
        // If the intersection point lies on BOTH segments, we have a valid collision.
        if (onSegment(p, A, B) && onSegment(p, C, D)) {
            intersection = p;
            return true;
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


    double distanceToNearestObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles) {
        
        // Initialize minDist with the maximum possible double value.
        // This ensures that the first comparison (dist < minDist) will always succeed.
        double minDist = std::numeric_limits<double>::max();

        // Iterate through every obstacle in the provided environment.
        for (const auto& obstacle : obstacles) {

            // --- Phase 1: Vertex Distance Check ---
            // We check the distance to every corner/vertex of the current obstacle.
            for (const auto& obstacle_point : obstacle.get_points()) {

                // Convert the obstacle's point format to a local Vertex for calculation.
                Vertex v(obstacle_point.x, obstacle_point.y);

                // Calculate Euclidean distance: sqrt((x2-x1)^2 + (y2-y1)^2)
                double dist = std::hypot(point.x - v.x, point.y - v.y);

                // If this vertex is closer than any previously checked point, updat
                if (dist < minDist) minDist = dist;
            }

            // --- Phase 2: Interior Check (Containment) ---
            // Even if a point is far from a vertex, it could be inside the obstacle polygon.
            // A distance of 0.0 is returned if the point is technically "colliding."
            if (pointInObstacle(point, obstacle)) return 0.0;
        }

        return minDist;
    }



    // It checks if the segment between p1 and p2 is free from obstacles with a minimum clearance
    bool isSegmentSafe(const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles, double min_clearance) {
        // --- Phase 1: Direct Intersection Check ---
        // Immediate exit if the mathematical line segment p1->p2 crosses any obstacle boundary.
        // This is a fast geometric intersection test (e.g., Line-Line or Line-Polygon).
        if (PlanningUtils::lineSegmentIntersectsObstacle(p1, p2, obstacles)) return false;
        
        // --- Phase 2: Clearance Buffer Validation ---

        // 1. Calculate the Euclidean distance of the segment using the Pythagorean theorem:
        double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);

        // 2. Determine the number of sampling points. 
        //    We divide the total distance by a fixed resolution (0.05 units).
        //    std::max(2, ...) ensures we check at least the start and end points
        double step_size = 0.05;
        int steps = std::max(2, (int)(dist / step_size));

        // 3. Linearly interpolate (LERP) along the segment.
        //    We iterate from i=0 (p1) to i=steps (p2).
        for (int i = 0; i <= steps; ++i) {

            // Compute the interpolation factor 't' (ranges from 0.0 to 1.0)
            double t = (double)i / steps;

            // Calculate the coordinates of the sampled point 'p' using the LERP formula:
            Vertex p(p1.x + t*(p2.x - p1.x), p1.y + t*(p2.y - p1.y));

            // 4. Distance Field Check:
            //    Calculate the shortest distance from the current point 'p' to any obstacle.
            //    If this distance is less than our threshold, the segment is "unsafe".
            if (PlanningUtils::distanceToNearestObstacle(p, obstacles) < min_clearance)
                return false;
        }
        return true;
    }



    bool isPointValid(double x, double y, const Map& map, double min_clearance) {
        // Wrap the raw coordinates into a Vertex object for geometric processing.
        Vertex p(x, y);

        // --- Step 1: Map Boundary Validation ---
        // Convert the map's border points into a simple polygon (vector of Vertices).
        // This defines the "legal" world where the robot/agent is allowed to exist.
        std::vector<Vertex> mapPoly;
        for(const auto& bp : map.borders.get_points()) mapPoly.push_back(toVertex(bp));
        
        // Check if the point 'p' lies within the outer map boundaries.
        // If the point is outside the perimeter, it is invalid regardless of obstacles.
        if (!pointInPolygon(p, mapPoly)) return false; 
        
        // --- Step 2: Floating Point & Precision Logic ---
        // If min_clearance is effectively zero, we perform a simpler 'Point-in-Polygon' 
        // check for obstacles rather than calculating exact Euclidean distances.
        // This is computationally cheaper.
        if (min_clearance <= 1e-5) {
            return !pointInAnyObstacle(p, map.obstacles.get_obstacles());
        }

        // --- Step 3: Clearance/Buffer Validation ---
        // Calculate the shortest Euclidean distance from point 'p' to the 
        // nearest edge or vertex of all obstacles in the map.
        double dist = distanceToNearestObstacle(p, map.obstacles.get_obstacles());

        // The point is valid only if its distance to the nearest obstacle 
        // is greater than or equal to the defined safety margin
        return (dist >= min_clearance);
    }
 



    // Integrate a position into the roadmap, connecting it to nearby nodes if possible
    void integratePosition(std::shared_ptr<Roadmap>& roadmap, const Vertex& pos, const std::vector<Obstacle>& obstacles, const std::string& label) {
    
        // --- 1. PROXIMITY DE-DUPLICATION ---
        // Prevent graph bloating by ensuring we don't add a vertex that is 
        // effectively at the same location as an existing one (5cm threshold).
        for(int i=0; i<roadmap->getNumVertices(); ++i) {
            if(roadmap->getVertex(i).distance(pos) < 0.05) return; 
        }

        // --- 2. FEASIBILITY CHECK ---
        // Ensure the point is within map bounds and not inside static geometry obstacles
        // before attempting any graph operations.
        if(!PlanningUtils::isPointValid(pos.x, pos.y, *(roadmap->getMap()))) {
            return; // Invalid position
        }
        
        // --- 3. GRAPH INSERTION ---
        // Officially add the vertex to the roadmap and retrieve its unique index.
        int newIdx = roadmap->addVertex(pos);
        
        // --- 4. NEIGHBOR DISCOVERY ---
        // Identify all existing nodes within a 15-meter radius.
        double search_radius = 15.0;                        // Search radius for neighbors
        std::vector<std::pair<double, int>> neighbors;
        
        // Find nearby vertices
        for (int i = 0; i < roadmap->getNumVertices(); ++i) {
            if (i == newIdx) continue;  // Do not connect a node to itself
            double d = roadmap->getVertex(i).distance(pos);     // Distance to new position
            if (d < search_radius) neighbors.push_back({d, i});
        }

        // Sort neighbors by distance (ascending) to prioritize connecting 
        // the robot to the closest available roadmap nodes first.
        std::sort(neighbors.begin(), neighbors.end());  // Sort by distance
        
        // --- 5. MULTI-PASS COLLISION RELAXATION ---
        // We try to connect using strict safety margins first (0.9m). 
        // If we fail to find enough connections (min 3), we reduce the margin
        // requirements (0.6m, then 0.3m) to allow passage through narrow spaces.
        std::vector<double> margins = {0.90, 0.60, 0.30};   // Safety margins in meters
        int connected_count = 0;

        for (double margin : margins) {

            // EXIT CONDITION A: If we successfully found at least 3 safe 
            // connections, we stop relaxing the margins to maintain maximum safety.
            if (connected_count >= 3) break;  
            
            // Attempt connections between new vertex and neighbors
            for (const auto& pair : neighbors) {

                // EXIT CONDITION B: Hard cap of 15 edges per node to maintain 
                // graph sparsity and search performance.
                if (connected_count >= 15) break;   
                
                int targetIdx = pair.second; 
                bool edgeExists = false;

                // --- 6. REDUNDANCY CHECK ---
                // Avoid adding duplicate edges between the same two nodes 
                // during different margin passes.
                for (const auto& e : roadmap->getEdges(newIdx)) {
                    if (e.targetVertex == targetIdx) {
                        edgeExists = true;
                        break;
                    }
                }
                if (edgeExists) continue;
                
                // --- 7. LINE-OF-SIGHT VALIDATION ---
                // Perform a sweep-volume check between the new position and the neighbor.
                bool possible = false;
                if (margin > 0.0)
                    // Check safety with the current buffer (margin).
                    possible = PlanningUtils::isSegmentSafe(pos, roadmap->getVertex(targetIdx), obstacles, margin);
                else
                    // Fallback to a simple ray-cast intersection if no margin is specified.
                    possible = !PlanningUtils::lineSegmentIntersectsObstacle(pos, roadmap->getVertex(targetIdx), obstacles);
                
                // --- 8. EDGE ESTABLISHMENT ---
                // If the path is clear, create a bidirectional connection.
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
        
        // --- Phase 0: Guard Clause ---
        // If the path is a single point or empty, no optimization is possible.
        if (rawPath.size() < 2) return rawPath;
        
        std::vector<int> optimized;         // Optimized final path
        optimized.push_back(rawPath[0]);    // Always keep the starting point
        int currentIdx = 0;                 // Pointer to the node we are currently trying to shortcut FROM
        
        // --- Phase 1: Greedy Shortcutting (String Pulling) ---
        // We iterate through the path to find the furthest "visible" node.
        while (currentIdx < rawPath.size() - 1) {
            bool shortcutFound = false;

            // Look ahead starting from the END of the path and moving backwards
            for (int i = rawPath.size() - 1; i > currentIdx + 1; --i) {
                const Vertex& vStart = roadmap.getVertex(rawPath[currentIdx]);
                const Vertex& vEnd = roadmap.getVertex(rawPath[i]);

                // Check if a straight line between current node and look-ahead node is safe
                if (PlanningUtils::isSegmentSafe(vStart, vEnd, obstacles, SAFETY_MARGIN)) {
                    optimized.push_back(rawPath[i]);    // Record the shortcut target
                    currentIdx = i;                     // Jump the cursor forward to the shortcut target
                    shortcutFound = true;
                    ROS_INFO("[PLANNING UTILS]: Shortcut found!");
                    break;  // Exit the inner loop to start searching from the new currentIdx
                }
            }

            // If no shortcut was found, we must move to the next immediate node
            if (!shortcutFound) {
                ROS_INFO("[PLANNING UTILS]: Shortcut not found!");
                optimized.push_back(rawPath[currentIdx + 1]);
                currentIdx++;
            }
        }
        
        // --- Phase 2: Proximity Filtering ---
        // Even if shortcuts are found, some nodes may be redundant due to spatial proximity.
        if (optimized.size() > 2) {
            std::vector<int> filtered;
            filtered.push_back(optimized[0]); // Keep the start

            for (size_t i = 1; i < optimized.size() - 1; ++i) {
                const Vertex& prev = roadmap.getVertex(filtered.back());
                const Vertex& curr = roadmap.getVertex(optimized[i]);

                // Only keep nodes that are at least 1.0 unit away from the previous kept node
                // This prevents the robot from trying to execute tiny, high-frequency movements.
                if (std::hypot(curr.x - prev.x, curr.y - prev.y) > 1.0) {
                    filtered.push_back(optimized[i]);
                }
            }

            filtered.push_back(optimized.back());// Always keep the goal point
            return filtered;
        }

        return optimized;
    }

}