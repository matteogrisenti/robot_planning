#include "libraries/combinatorial_planning/shortest_path_roadmap.h"
#include "libraries/planning_utils.h"

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



std::shared_ptr<Roadmap> shortestPathRoadmap(const Map& map, const double padding) {
    std::shared_ptr<Roadmap> roadmap = std::make_shared<Roadmap>();
    roadmap->setMap(&map);

    // 0. Pre-process Obstacles with Padding
    // We create a local vector of obstacles. If padding > 0, these will be the inflated versions.
    // If padding == 0, these are copies of the originals.
    // We will use THIS vector for both Roadmap Vertices and Collision Checks.
    std::vector<Obstacle> effective_obstacles;
    
    // We need to keep the point data alive if Obstacle stores pointers, 
    // but the Obstacle struct in provided header copies points into a vector.
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

        // Store starting index for this obstacle's vertices
        int start_id = roadmap->getNumVertices();

        for (size_t i = 0; i < n; ++i) {
            // The check is made only if padding is not zero 
            // (in thisa case the point is guaranteed to be not valid)
            if(padding == 0 || PlanningUtils::isPointValid(pts[i].x, pts[i].y, map) == true) {
                roadmap->addVertex(PlanningUtils::toVertex(pts[i]));
            }
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

        // 2. Aggiungi esplicitamente Vittime e Gate come nodi
    for (const auto& v : map.victims.get_victims()) {
        roadmap->addVertex(Vertex(v.get_center().x, v.get_center().y));
    }
    if (!map.gates.get_gates().empty()) {
        auto g = map.gates.get_gates()[0].get_position();
        roadmap->addVertex(Vertex(g.x, g.y));
    }

    // 3. Loop di Visibilità (modificato per includere tutti i nodi appena aggiunti)
    int total_nodes = roadmap->getNumVertices();

    // 3. Connect Visible Vertices (Bitangents)
    // Iterate through all unique pairs of vertices to form the Visibility Graph.
    // This connects any two vertices (u, v) if the line segment uv lies entirely in Free Space.


    // Prepare map border vertices once for efficiency in the loop
    std::vector<Vertex> map_border_vertices;
    for (const auto& p : map.borders.get_points()) {
        map_border_vertices.push_back(PlanningUtils::toVertex(p));
    }

    for (int i = 0; i < total_nodes; ++i) {
        for (int j = i + 1; j < total_nodes; ++j) {

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
            if (PlanningUtils::lineSegmentIntersectsObstacle(v1, v2, effective_obstacles)) {
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



// Helper to normalize a vector
Point normalizeVector(const Point& p) {
    double len = std::hypot(p.x, p.y);
    if (len < 1e-9) return {0, 0, 0};
    return {p.x / len, p.y / len, 0};
}

Point outwardNormal(const Point& v, bool ccw) {
    if (ccw) {
        // right normal
        return { v.y, -v.x, 0 };
    } else {
        // left normal
        return { -v.y, v.x, 0 };
    }
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
            padded_poly.push_back({
                p_curr.x + n1.x * padding,
                p_curr.y + n1.y * padding,
                0
            });
            continue;
        }

        Point b = {bisector.x / len, bisector.y / len, 0};
        double cos_half = n1.x * b.x + n1.y * b.y;
        cos_half = std::max(cos_half, 0.01);

        double dist = padding / cos_half;

        padded_poly.push_back({
            p_curr.x + b.x * dist,
            p_curr.y + b.y * dist,
            0
        });
    }

    return padded_poly;
}
