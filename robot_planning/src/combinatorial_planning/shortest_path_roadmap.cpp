#include "combinatorial_planning/shortest_path_roadmap.h"
#include "combinatorial_planning/planning_utils.h"
#include "map/map_data_structures.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

// Constants
const double EPSILON = 1e-5;

// Helper for Reflex Vertex detection (Cross Product)
double crossProduct(const Point& a, const Point& b, const Point& c) {
    return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
}



std::shared_ptr<Roadmap> generateShortestPathRoadmap(const Map& map) {
    std::shared_ptr<Roadmap> roadmap = std::make_shared<Roadmap>();
    roadmap->setMap(&map);

    // 1. Add All Obstacle Vertices as Edges
    const auto& obstacles = map.obstacles.get_obstacles();
    int vertex_offset = 0; // To track IDs for edge creation

    for (const auto& obs : obstacles) {
        const std::vector<Point>& pts = obs.get_points();
        size_t n = pts.size();
        if (n < 2) continue;

        // Store starting index for this obstacle's vertices
        int start_id = roadmap->getNumVertices();

        for (size_t i = 0; i < n; ++i) {
            roadmap->addVertex(PlanningUtils::toVertex(pts[i]));
        }

        // Add edges between consecutive vertices of the obstacle
        for (size_t i = 0; i < n; ++i) {
            int u = start_id + i;
            int v = start_id + ((i + 1) % n);
            roadmap->addEdge(u, v);
        }
    }

    // 2. Add Reflex Vertices from Map Borders
    const std::vector<Point>& b_pts = map.borders.get_points();
    if (b_pts.size() >= 3) {
        // Determine winding order (Shoelace formula) to distinguish convex vs reflex
        double area = 0;
        for (size_t i = 0; i < b_pts.size(); ++i) {
            size_t j = (i + 1) % b_pts.size();
            area += (b_pts[i].x * b_pts[j].y - b_pts[j].x * b_pts[i].y);
        }
        bool isCCW = (area > 0);

        for (size_t i = 0; i < b_pts.size(); ++i) {
            const Point& prev = b_pts[(i - 1 + b_pts.size()) % b_pts.size()];
            const Point& curr = b_pts[i];
            const Point& next = b_pts[(i + 1) % b_pts.size()];

            double cp = crossProduct(prev, curr, next);

            // Logic: Detect Reflex (concave) corners where paths might pivot.
            // CCW: Right Turn (< 0) is Reflex.
            // CW:  Left Turn (> 0) is Reflex.
            bool isReflex = isCCW ? (cp < -EPSILON) : (cp > EPSILON);

            if (isReflex) {
                roadmap->addVertex(PlanningUtils::toVertex(curr));
            }
        }
    }

    // 3. Connect Visible Vertices (Bitangents)
    // Iterate through all unique pairs of vertices to form the Visibility Graph.
    // This connects any two vertices (u, v) if the line segment uv lies entirely in Free Space.
    int num_vertices = roadmap->getNumVertices();

    // Prepare map border vertices once for efficiency in the loop
    std::vector<Vertex> map_border_vertices;
    for (const auto& p : map.borders.get_points()) {
        map_border_vertices.push_back(PlanningUtils::toVertex(p));
    }

    for (int i = 0; i < num_vertices; ++i) {
        for (int j = i + 1; j < num_vertices; ++j) {

            Vertex v1 = roadmap->getVertex(i);
            Vertex v2 = roadmap->getVertex(j);

            // Optimization: Skip extremely short segments (floating point noise)
            double dx = v1.x - v2.x;
            double dy = v1.y - v2.y;
            double dist_sq = dx*dx + dy*dy;
            
            if (dist_sq < (1e-5 * 1e-5)) continue; 

            // ---------------- VISIBILITY CHECK ----------------
            bool visible = true;

            // A. Check strict intersection with any obstacle edges
            if (PlanningUtils::lineSegmentIntersectsObstacle(v1, v2, map.obstacles.get_obstacles())) {
                visible = false;
            }

            // ---------------- ADD EDGE ----------------
            if (visible) {
                // Calculate Euclidean distance for edge weight
                roadmap->addEdge(i, j, true);
            }
        }
    }

    return roadmap;
}