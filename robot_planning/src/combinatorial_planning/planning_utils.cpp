#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "combinatorial_planning/planning_utils.h"

namespace PlanningUtils {

    // --- Geometry Implementation ---

    bool pointInPolygon(const Vertex& point, const std::vector<Vertex>& polygon) {
        bool inside = false;
        size_t n = polygon.size();
        
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const Vertex& v1 = polygon[i];
            const Vertex& v2 = polygon[j];
            
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

    // --- Validation & Conversion (Richiesto da SPR) ---

    Vertex toVertex(const Point& p) {
        return Vertex(p.x, p.y);
    }

    bool isPointValid(double x, double y, const Map& map) {
        Vertex p(x, y);

        // 1. Check borders
        std::vector<Vertex> borderPoly;
        for(const auto& bp : map.borders.get_points()) {
            borderPoly.push_back(Vertex(bp.x, bp.y));
        }
        if (!pointInPolygon(p, borderPoly)) return false;

        // 2. Check obstacles
        if (pointInAnyObstacle(p, map.obstacles.get_obstacles())) return false;

        return true;
    }

    // --- Legacy Helper ---
    void connectTargetsToRoadmap(std::shared_ptr<Roadmap> roadmap, const Map& map) {
        // Logica ora integrata.
    }
}