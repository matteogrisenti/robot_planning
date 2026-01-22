#include <limits>
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

#include "libraries/planning_utils.h"

namespace PlanningUtils {

    bool pointInPolygon(const Vertex& point, const std::vector<Vertex>& polygon) {
        bool inside = false;
        size_t n = polygon.size();
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const Vertex& v1 = polygon[i];
            const Vertex& v2 = polygon[j];

            bool intersects = ((v1.y > point.y) != (v2.y > point.y)) &&
                              (point.x < (v2.x - v1.x) * (point.y - v1.y) / (v2.y - v1.y) + v1.x);
            
            if (intersects) inside = !inside;
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

    Vertex toVertex(const Point& p) { 
        return Vertex(p.x, p.y); 
    }

    bool getSegmentIntersection(const Vertex& A, const Vertex& B, 
                                const Vertex& C, const Vertex& D, 
                                Vertex& intersection) {

        double a1 = B.y - A.y;
        double b1 = A.x - B.x;
        double c1 = a1 * A.x + b1 * A.y;
        double a2 = D.y - C.y;
        double b2 = C.x - D.x;
        double c2 = a2 * C.x + b2 * C.y;
        double determinant = a1 * b2 - a2 * b1;
        if (std::abs(determinant) < 1e-9) return false; 
        double x = (b2 * c1 - b1 * c2) / determinant;
        double y = (a1 * c2 - a2 * c1) / determinant;
        
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

    bool lineSegmentIntersectsObstacle(
        const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles) {
       
        for (const auto& obs : obstacles) {
            const std::vector<Point>& pts = obs.get_points();
            size_t n = pts.size();
            if (n < 2) continue;
            for (size_t i = 0; i < n; ++i) {
                Vertex vC = toVertex(pts[i]);
                Vertex vD = toVertex(pts[(i + 1) % n]);
                double d1 = std::hypot(p1.x - vC.x, p1.y - vC.y);
                double d2 = std::hypot(p1.x - vD.x, p1.y - vD.y);
                double d3 = std::hypot(p2.x - vC.x, p2.y - vC.y);
                double d4 = std::hypot(p2.x - vD.x, p2.y - vD.y);
                
                if (d1 < 1e-5 || d2 < 1e-5 || d3 < 1e-5 || d4 < 1e-5) {
                    continue;
                }

                Vertex intersection;
                if (getSegmentIntersection(p1, p2, vC, vD, intersection)) {
                    return true;
                }
            }
        }
        Vertex mid;
        mid.x = (p1.x + p2.x) / 2.0;
        mid.y = (p1.y + p2.y) / 2.0;
        if (pointInAnyObstacle(mid, obstacles)) {
            return true; 
        }

        return false;
    }


    double distanceToNearestObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles) {
        double minDist = std::numeric_limits<double>::max();

        for (const auto& obstacle : obstacles) {
            for (const auto& obstacle_point : obstacle.get_points()) {
                Vertex v(obstacle_point.x, obstacle_point.y);
                double dist = std::hypot(point.x - v.x, point.y - v.y);
                if (dist < minDist) minDist = dist;
            }
            if (pointInObstacle(point, obstacle)) return 0.0;
        }

        return minDist;
    }

    bool isSegmentSafe(const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles, double min_clearance) {
        if (PlanningUtils::lineSegmentIntersectsObstacle(p1, p2, obstacles)) return false;
        double dist = std::hypot(p2.x - p1.x, p2.y - p1.y);
        double step_size = 0.05;
        int steps = std::max(2, (int)(dist / step_size));
        for (int i = 0; i <= steps; ++i) {
            double t = (double)i / steps;
            Vertex p(p1.x + t*(p2.x - p1.x), p1.y + t*(p2.y - p1.y));
            if (PlanningUtils::distanceToNearestObstacle(p, obstacles) < min_clearance)
                return false;
        }
        return true;
    }

    bool isPointValid(double x, double y, const Map& map, double min_clearance) {
        Vertex p(x, y);
        std::vector<Vertex> mapPoly;
        for(const auto& bp : map.borders.get_points()) mapPoly.push_back(toVertex(bp));
        if (!pointInPolygon(p, mapPoly)) return false; 
        if (min_clearance <= 1e-5) {
            return !pointInAnyObstacle(p, map.obstacles.get_obstacles());
        }
        double dist = distanceToNearestObstacle(p, map.obstacles.get_obstacles());
        return (dist >= min_clearance);
    }
 
    void integratePosition(std::shared_ptr<Roadmap>& roadmap, const Vertex& pos, const std::vector<Obstacle>& obstacles, const std::string& label) {
        for(int i=0; i<roadmap->getNumVertices(); ++i) {
            if(roadmap->getVertex(i).distance(pos) < 0.05) return; 
        }
        if(!PlanningUtils::isPointValid(pos.x, pos.y, *(roadmap->getMap()))) {
            return; 
        }
        int newIdx = roadmap->addVertex(pos);
        double search_radius = 15.0;                        // Search radius for neighbors
        std::vector<std::pair<double, int>> neighbors;
        for (int i = 0; i < roadmap->getNumVertices(); ++i) {
            if (i == newIdx) continue;  
            double d = roadmap->getVertex(i).distance(pos);     
            if (d < search_radius) neighbors.push_back({d, i});
        }
        std::sort(neighbors.begin(), neighbors.end());  
        std::vector<double> margins = {0.90, 0.60, 0.30};   
        int connected_count = 0;

        for (double margin : margins) {
            if (connected_count >= 3) break;  
            for (const auto& pair : neighbors) {
                if (connected_count >= 15) break;   
                int targetIdx = pair.second; 
                bool edgeExists = false;
                for (const auto& e : roadmap->getEdges(newIdx)) {
                    if (e.targetVertex == targetIdx) {
                        edgeExists = true;
                        break;
                    }
                }
                if (edgeExists) continue;
                bool possible = false;
                if (margin > 0.0)
                    possible = PlanningUtils::isSegmentSafe(pos, roadmap->getVertex(targetIdx), obstacles, margin);
                else
                    possible = !PlanningUtils::lineSegmentIntersectsObstacle(pos, roadmap->getVertex(targetIdx), obstacles);
                if (possible) {
                    roadmap->addEdge(newIdx, targetIdx, pair.first);
                    roadmap->addEdge(targetIdx, newIdx, pair.first);
                    connected_count++;
                }
            }
        }
    }


    std::vector<int> optimizePath(const std::vector<int>& rawPath, const Roadmap& roadmap, const std::vector<Obstacle>& obstacles, double SAFETY_MARGIN) {
        if (rawPath.size() < 2) return rawPath;

        std::vector<int> optimized;         
        optimized.push_back(rawPath[0]);    
        int currentIdx = 0;                 
        while (currentIdx < rawPath.size() - 1) {
            bool shortcutFound = false;
            for (int i = rawPath.size() - 1; i > currentIdx + 1; --i) {
                const Vertex& vStart = roadmap.getVertex(rawPath[currentIdx]);
                const Vertex& vEnd = roadmap.getVertex(rawPath[i]);

                if (PlanningUtils::isSegmentSafe(vStart, vEnd, obstacles, SAFETY_MARGIN)) {
                    optimized.push_back(rawPath[i]);    
                    currentIdx = i;                     
                    shortcutFound = true;
                    //ROS_INFO("[PLANNING UTILS]: Shortcut found!");
                    break;  
                }
            }
            if (!shortcutFound) {
                //ROS_INFO("[PLANNING UTILS]: Shortcut not found!");
                optimized.push_back(rawPath[currentIdx + 1]);
                currentIdx++;
            }
        }
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