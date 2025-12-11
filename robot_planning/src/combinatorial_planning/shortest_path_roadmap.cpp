#include "combinatorial_planning/shortest_path_roadmap.h"

#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

// Use OpenCV for handy point arithmetic and basic geometric types if available, 
// otherwise we use standard math. Assuming OpenCV is linked as per your CMake.
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// =========================================================
// GEOMETRIC HELPER FUNCTIONS
// =========================================================

// Constants
const double EPSILON = 1e-9;
const double PI = 3.14159265358979323846;

struct GeomPoint {
    double x, y;
};

// Cross product of (b-a) and (c-b). 
// Returns > 0 for Left turn, < 0 for Right turn, 0 for Collinear.
double crossProduct(const GeomPoint& a, const GeomPoint& b, const GeomPoint& c) {
    return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
}

// Check if segment AB intersects segment CD strictly (excluding endpoints).
bool segmentsIntersect(const GeomPoint& a, const GeomPoint& b, 
                       const GeomPoint& c, const GeomPoint& d) {
    auto ccw = [](const GeomPoint& p1, const GeomPoint& p2, const GeomPoint& p3) {
        return (p3.y - p1.y) * (p2.x - p1.x) > (p2.y - p1.y) * (p3.x - p1.x);
    };
    
    return (ccw(a, c, d) != ccw(b, c, d)) && (ccw(a, b, c) != ccw(a, b, d));
}

// Check if a point is on a segment (roughly)
bool isPointOnSegment(const GeomPoint& p, const GeomPoint& a, const GeomPoint& b) {
    double cross = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    if (std::abs(cross) > EPSILON) return false; // Not collinear

    double dot = (p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y);
    double len2 = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
    
    return dot >= -EPSILON && dot <= len2 + EPSILON;
}

// Distance between points
double dist(const GeomPoint& a, const GeomPoint& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

// Check visibility between u and v
// 1. Must not intersect strictly any obstacle edge.
// 2. Midpoint must be valid (in C-free) to handle non-convex cases or same-polygon diagonals.
bool isVisible(const GeomPoint& u, const GeomPoint& v, const Map& map) {
    // 1. Check Intersection with all Map Borders
    // Note: Map Borders usually define the enclosure. A ray should be INSIDE.
    // However, basic visibility checks if the segment crosses a wall.
    
    // Check Borders Edges
    const std::vector<Point>& b_pts = map.borders.get_points();
    if (b_pts.size() > 1) {
        for (size_t i = 0; i < b_pts.size(); ++i) {
            size_t next = (i + 1) % b_pts.size();
            GeomPoint p1 = {b_pts[i].x, b_pts[i].y};
            GeomPoint p2 = {b_pts[next].x, b_pts[next].y};

            // Ignore if the segment shares an endpoint with the edge
            if (dist(u, p1) < EPSILON || dist(u, p2) < EPSILON || 
                dist(v, p1) < EPSILON || dist(v, p2) < EPSILON) {
                continue; 
            }

            if (segmentsIntersect(u, v, p1, p2)) return false;
        }
    }

    // Check Obstacle Edges
    for (const auto& obs : map.obstacles.get_obstacles()) {
        const std::vector<Point>& pts = obs.get_points();
        if (pts.size() < 2) continue;

        for (size_t i = 0; i < pts.size(); ++i) {
            size_t next = (i + 1) % pts.size();
            GeomPoint p1 = {pts[i].x, pts[i].y};
            GeomPoint p2 = {pts[next].x, pts[next].y};

            // Ignore if segment shares endpoint
            if (dist(u, p1) < EPSILON || dist(u, p2) < EPSILON || 
                dist(v, p1) < EPSILON || dist(v, p2) < EPSILON) {
                continue; 
            }

            if (segmentsIntersect(u, v, p1, p2)) return false;
        }
    }

    // 2. Midpoint Check
    // This ensures we aren't crossing through the INTERIOR of an obstacle 
    // (e.g., connecting two vertices of the same convex obstacle via the inside)
    // or going outside the map.
    GeomPoint mid;
    mid.x = (u.x + v.x) / 2.0;
    mid.y = (u.y + v.y) / 2.0;

    // Check if Midpoint is INSIDE any obstacle (invalid)
    for (const auto& obs : map.obstacles.get_obstacles()) {
        // Use OpenCV for robust point-in-polygon
        std::vector<cv::Point2f> poly;
        for(const auto& p : obs.get_points()) poly.emplace_back((float)p.x, (float)p.y);
        
        // pointPolygonTest: >0 inside, <0 outside, 0 on edge.
        // If strictly inside (> EPS), it's blocked.
        if (cv::pointPolygonTest(poly, cv::Point2f((float)mid.x, (float)mid.y), false) > 0) {
            return false;
        }
    }

    // Check if Midpoint is OUTSIDE the map borders (invalid)
    std::vector<cv::Point2f> borderPoly;
    for(const auto& p : map.borders.get_points()) borderPoly.emplace_back((float)p.x, (float)p.y);
    if (cv::pointPolygonTest(borderPoly, cv::Point2f((float)mid.x, (float)mid.y), false) < 0) {
        return false;
    }

    return true;
}

// =========================================================
// MAIN ALGORITHM
// =========================================================

Roadmap generateShortestPathRoadmap(const Map& map) {
    Roadmap roadmap;
    roadmap.setMap(&map);

    // Structure to keep track of created Roadmap IDs
    // We store candidates as {x, y, original_polygon_index}
    struct Candidate {
        GeomPoint pt;
        int roadmapId;
    };
    std::vector<Candidate> candidates;

    // 1. Identify REFLEX Vertices from OBSTACLES
    // "All vertices of a convex polygon are reflex vertices" (relative to C-Free)
    const auto& obstacles = map.obstacles.get_obstacles();
    for (const auto& obs : obstacles) {
        const auto& pts = obs.get_points();
        for (const auto& p : pts) {
            int id = roadmap.addVertex(Vertex(p.x, p.y));
            candidates.push_back({ {p.x, p.y}, id });
        }
    }

    // 2. Identify REFLEX Vertices from BORDERS
    // A border vertex is reflex if the interior angle (in C-free) is > 180 (Pi).
    // Assuming CCW winding for the border:
    // Left Turn = Convex (Interior < 180). Right Turn = Concave/Reflex (Interior > 180).
    const auto& b_pts = map.borders.get_points();
    if (b_pts.size() >= 3) {
        // Determine winding order (Shoelace)
        double area = 0;
        for (size_t i = 0; i < b_pts.size(); ++i) {
            size_t j = (i + 1) % b_pts.size();
            area += (b_pts[i].x * b_pts[j].y - b_pts[j].x * b_pts[i].y);
        }
        bool isCCW = (area > 0);

        for (size_t i = 0; i < b_pts.size(); ++i) {
            size_t prev = (i - 1 + b_pts.size()) % b_pts.size();
            size_t next = (i + 1) % b_pts.size();

            GeomPoint p_prev = {b_pts[prev].x, b_pts[prev].y};
            GeomPoint p_curr = {b_pts[i].x, b_pts[i].y};
            GeomPoint p_next = {b_pts[next].x, b_pts[next].y};

            double cp = crossProduct(p_prev, p_curr, p_next);

            // Logic:
            // If CCW: Left Turn (>0) is Convex (standard corner). Right Turn (<0) is Reflex.
            // If CW: Right Turn (<0) is Convex. Left Turn (>0) is Reflex.
            bool isReflex = false;
            if (isCCW) {
                if (cp < -EPSILON) isReflex = true; 
            } else {
                if (cp > EPSILON) isReflex = true;
            }

            // Note: If you want the roadmap to reach into corners of the room 
            // (even if not reflex), you might add ALL border vertices. 
            // But strict Shortest Path logic uses only reflex vertices.
            if (isReflex) {
                int id = roadmap.addVertex(Vertex(p_curr.x, p_curr.y));
                candidates.push_back({ p_curr, id });
            }
        }
    }

    // 3. Add Edges between "Consecutive Reflex Vertices" 
    // (Existing edges of Obstacles)
    // We iterate obstacles again to link their vertices if both are in our candidate list.
    // Since we added ALL obstacle vertices, we just link p[i] to p[i+1].
    
    // We need to map coordinates back to IDs efficiently, or just track indices.
    // Since we added them sequentially in step 1:
    int current_id_counter = 0;
    for (const auto& obs : obstacles) {
        int n = obs.get_points().size();
        if (n < 2) continue;
        
        // The first vertex of this obstacle starts at current_id_counter
        int start_idx = current_id_counter;
        
        for (int i = 0; i < n; ++i) {
            int u = start_idx + i;
            int v = start_idx + ((i + 1) % n);
            
            // Check if edge is valid (it is, because it's an obstacle boundary)
            // Add edge to roadmap
            roadmap.addEdge(u, v, true); 
        }
        current_id_counter += n;
    }

    // Note: We are not explicitly adding Border edges here because we only 
    // picked the Reflex vertices of the border. Border edges connecting 
    // two reflex vertices will be caught by the "Visibility" check below.

    // 4. Add "Bitangent" Edges (Visibility)
    // Connect any two vertices in the graph if they are mutually visible.
    // This creates the visibility graph.
    
    int num_v = roadmap.getNumVertices();
    for (int i = 0; i < num_v; ++i) {
        for (int j = i + 1; j < num_v; ++j) {
            
            // Optimization: If they already have an edge (from step 3), skip
            // (Roadmap usually handles duplicates or we check here)
            // Simpler: just check geometry.

            const Vertex& v1_r = roadmap.getVertex(i);
            const Vertex& v2_r = roadmap.getVertex(j);
            
            GeomPoint p1 = {v1_r.x, v1_r.y};
            GeomPoint p2 = {v2_r.x, v2_r.y};

            if (isVisible(p1, p2, map)) {
                // Add edge. (Check for duplicates if Roadmap doesn't)
                // Assuming roadmap.addEdge handles duplicates or is okay with them.
                roadmap.addEdge(i, j, true);
            }
        }
    }

    return roadmap;
}