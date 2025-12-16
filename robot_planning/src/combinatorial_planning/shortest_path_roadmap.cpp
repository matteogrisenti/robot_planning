#include "combinatorial_planning/shortest_path_roadmap.h"
#include "combinatorial_planning/planning_utils.h"
#include "map/map_data_structures.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <iostream> 

// Constants
const double EPSILON = 1e-5;

// Helper for Reflex Vertex detection (Cross Product)
double crossProduct(const Point& a, const Point& b, const Point& c) {
    return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
}

// Forward declaration of helpers
std::vector<Point> applyPaddingToPolygon(const std::vector<Point>& poly, double padding);


std::shared_ptr<Roadmap> generateShortestPathRoadmap(const Map& map, const double padding) {
    std::shared_ptr<Roadmap> roadmap = std::make_shared<Roadmap>();
    roadmap->setMap(&map);

    // 0. Pre-process Obstacles with Padding
    std::vector<Obstacle> effective_obstacles;
    
    for (const auto& obs : map.obstacles.get_obstacles()) {
        if (padding > EPSILON) {
            std::vector<Point> padded_pts = applyPaddingToPolygon(obs.get_points(), padding);
            effective_obstacles.emplace_back(&padded_pts, obs.get_radius() + (float)padding);
        } else {
            effective_obstacles.push_back(obs);
        }
    }

    // 1. Add Reflex Vertices from Obstacles
    for (const auto& obs : effective_obstacles) {
        const std::vector<Point>& pts = obs.get_points();
        size_t n = pts.size();
        if (n < 2) continue;

        int start_id = roadmap->getNumVertices();

        for (size_t i = 0; i < n; ++i) {
            if(padding == 0 || PlanningUtils::isPointValid(pts[i].x, pts[i].y, map)) {
                roadmap->addVertex(PlanningUtils::toVertex(pts[i]));
            }
        }

        // Add edges between consecutive vertices (polygon boundary)
        int current_nodes = roadmap->getNumVertices();
        int added_count = current_nodes - start_id;
        
        // Only connect if we actually added vertices for this obstacle
        if (added_count > 1) { 
            for (int i = 0; i < added_count; ++i) {
                int u = start_id + i;
                int v = start_id + ((i + 1) % added_count);
                roadmap->addEdge(u, v);
            }
        }
    }

    // 2. Add Reflex Vertices from Map Borders
    const std::vector<Point>& b_pts = map.borders.get_points();
    if (b_pts.size() >= 3) {
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
            bool isReflex = isCCW ? (cp < -EPSILON) : (cp > EPSILON);

            if (isReflex) {
                roadmap->addVertex(PlanningUtils::toVertex(curr));
            }
        }
    }

    // ========================================================
    // 3. INJECT TARGETS (Start, Gates, Victims)
    // ========================================================
    // Questi punti devono essere parte del grafo per essere connessi
    
    // Start
    roadmap->addVertex(PlanningUtils::toVertex(map.start.get_position()));

    // Gates
    for (const auto& gate : map.gates.get_gates()) {
        roadmap->addVertex(PlanningUtils::toVertex(gate.get_position()));
    }

    // Victims
    for (const auto& victim : map.victims.get_victims()) {
        roadmap->addVertex(PlanningUtils::toVertex(victim.get_center()));
    }
    // ========================================================


    // 4. Connect Visible Vertices (Visibility Graph Construction)
    // O(N^2) check: connect all pairs (u,v) if visible
    int num_vertices = roadmap->getNumVertices();

    for (int i = 0; i < num_vertices; ++i) {
        for (int j = i + 1; j < num_vertices; ++j) {

            Vertex v1 = roadmap->getVertex(i);
            Vertex v2 = roadmap->getVertex(j);

            // Optimization: Skip extremely short segments
            double dx = v1.x - v2.x;
            double dy = v1.y - v2.y;
            if ((dx*dx + dy*dy) < 1e-10) continue; 

            // ---------------- VISIBILITY CHECK ----------------
            // Check intersection with any obstacle edge (using effective/padded obstacles)
            bool visible = !PlanningUtils::lineSegmentIntersectsObstacle(v1, v2, effective_obstacles);

            // ---------------- ADD EDGE ----------------
            if (visible) {
                roadmap->addEdge(i, j, true);
            }
        }
    }

    return roadmap;
}

// --- Internal Helper Implementations ---

Point normalizeVector(const Point& p) {
    double len = std::hypot(p.x, p.y);
    if (len < 1e-9) return {0, 0, 0};
    return {p.x / len, p.y / len, 0};
}

Point outwardNormal(const Point& v, bool ccw) {
    if (ccw) return { v.y, -v.x, 0 }; // Right normal
    else return { -v.y, v.x, 0 };     // Left normal
}

bool isCCW(const std::vector<Point>& poly) {
    double area = 0.0;
    for (size_t i = 0; i < poly.size(); ++i) {
        size_t j = (i + 1) % poly.size();
        area += (poly[i].x * poly[j].y - poly[j].x * poly[i].y);
    }
    return area > 0;
}

std::vector<Point> applyPaddingToPolygon(const std::vector<Point>& poly, double padding) {
    std::vector<Point> padded_poly;
    size_t n = poly.size();
    if (n < 3) return poly;

    bool ccw = isCCW(poly);

    for (size_t i = 0; i < n; ++i) {
        Point p_prev = poly[(i + n - 1) % n];
        Point p_curr = poly[i];
        Point p_next = poly[(i + 1) % n];

        Point v1 = {p_curr.x - p_prev.x, p_curr.y - p_prev.y, 0};
        Point v2 = {p_next.x - p_curr.x, p_next.y - p_curr.y, 0};

        Point n1 = normalizeVector(outwardNormal(v1, ccw));
        Point n2 = normalizeVector(outwardNormal(v2, ccw));

        Point bisector = {n1.x + n2.x, n1.y + n2.y, 0};
        double len = std::hypot(bisector.x, bisector.y);

        if (len < 1e-6) {
            padded_poly.push_back({p_curr.x + n1.x * padding, p_curr.y + n1.y * padding, 0});
            continue;
        }

        Point b = {bisector.x / len, bisector.y / len, 0};
        double cos_half = n1.x * b.x + n1.y * b.y;
        cos_half = std::max(cos_half, 0.01); 

        double dist = padding / cos_half;
        padded_poly.push_back({p_curr.x + b.x * dist, p_curr.y + b.y * dist, 0});
    }

    return padded_poly;
}