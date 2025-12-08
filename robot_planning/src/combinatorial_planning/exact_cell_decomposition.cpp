#include "combinatorial_planning/exact_cell_decomposition.h"
#include "combinatorial_planning/planning_utils.h" 
#include <algorithm>
#include <set>
#include <limits>
#include <iostream>

namespace ExactDecomposition {

    // --- Internal Helpers ---

    struct Point2D { double x, y; };

    struct Segment {
        Point2D p1, p2;
        
        // Calculate Y at a specific X on this segment
        double getYAtX(double x) const {
            // Handle vertical segments safely (though they usually define the walls, not top/bot)
            if (std::abs(p2.x - p1.x) < 1e-9) return std::max(p1.y, p2.y); 
            
            double t = (x - p1.x) / (p2.x - p1.x);
            return p1.y + t * (p2.y - p1.y);
        }
    };

    Point2D toPoint2D(const Point& p) { return {p.x, p.y}; }

    // Check if a point is in valid free space (inside borders, outside obstacles)
    bool isFreeSpace(const Vertex& p, const Map& map) {
        // 1. Must be inside Map Borders
        std::vector<Vertex> borderPoly;
        for(const auto& bp : map.borders.get_points()) borderPoly.push_back(Vertex(bp.x, bp.y));
        if (!PlanningUtils::pointInPolygon(p, borderPoly)) return false;

        // 2. Must NOT be inside any Obstacle
        if (PlanningUtils::pointInAnyObstacle(p, map.obstacles.get_obstacles())) return false;

        return true;
    }

    // --- Main Function ---

    std::shared_ptr<Roadmap> exactCellDecomposition(const Map& map) {
        auto roadmap = std::make_shared<Roadmap>();
        roadmap->setMap(&map);

        // 1. Compute Decomposition
        std::vector<Trapezoid> trapezoids = computeTrapezoidalDecomposition(map);
        // Link it to the roadmap for plotting
        roadmap->debugTrapezoids = std::make_shared<std::vector<Trapezoid>>(trapezoids);

        // 2. Compute Adjacency (Neighbors)
        connectAdjacentTrapezoids(trapezoids);

        // 3. Build Roadmap Graph
        // Add all centroids as nodes
        std::vector<int> trapIdToNodeId(trapezoids.size());
        
        for (size_t i = 0; i < trapezoids.size(); ++i) {
            trapIdToNodeId[i] = roadmap->addVertex(trapezoids[i].center);
        }

        // Iterate over all trapezoids to connect them
        for (size_t i = 0; i < trapezoids.size(); ++i) {
            const auto& t1 = trapezoids[i];

            for (int neighborIdx : trapezoids[i].neighbors) {
                // Avoid adding double edges (undirected graph)
                if (neighborIdx <= (int)i) continue;

                const auto& t2 = trapezoids[neighborIdx];
                
                // 1. Identify Shared Boundary X
                bool t1IsLeft = std::abs(t1.rightX - t2.leftX) < 1e-6;
                double sharedX = t1IsLeft ? t1.rightX : t1.leftX;

                // 2. Identify Shared Boundary Y-Interval
                // Get t1's Y range at the shared X
                double t1_y_high = t1IsLeft ? t1.topRightY : t1.topLeftY;
                double t1_y_low  = t1IsLeft ? t1.bottomRightY : t1.bottomLeftY;

                // Get t2's Y range at the shared X
                double t2_y_high = t1IsLeft ? t2.topLeftY : t2.topRightY;
                double t2_y_low  = t1IsLeft ? t2.bottomLeftY : t2.bottomRightY;

                // Calculate the geometric overlap
                double overlapStart = std::max(t1_y_low, t2_y_low);
                double overlapEnd   = std::min(t1_y_high, t2_y_high);

                // 3. Create Gateway Vertex
                // We place a vertex exactly in the middle of the "door" between trapezoids
                double midY = (overlapStart + overlapEnd) / 2.0;
                Vertex gateway(sharedX, midY);

                int gatewayNodeId = roadmap->addVertex(gateway);

                // 4. Connect: Center1 -> Gateway -> Center2
                roadmap->addEdge(trapIdToNodeId[i], gatewayNodeId, true);
                roadmap->addEdge(gatewayNodeId, trapIdToNodeId[neighborIdx], true);
            }
        }

        return roadmap;
    }

    // --- Decomposition Logic ---

    std::vector<Trapezoid> computeTrapezoidalDecomposition(const Map& map) {
        std::vector<Trapezoid> result;
        std::vector<Segment> allSegments;
        std::set<double> x_events;

        // Helper to decompose polygons into segments and events
        auto processPolygon = [&](const std::vector<Point>& pts) {
            if (pts.empty()) return;
            for (size_t i = 0; i < pts.size(); ++i) {
                Point2D p1 = toPoint2D(pts[i]);
                Point2D p2 = toPoint2D(pts[(i + 1) % pts.size()]);
                
                x_events.insert(p1.x);
                
                // Sort segment X to simplify intersection logic
                if (p1.x > p2.x) std::swap(p1, p2);
                
                // Only keep non-vertical segments for Top/Bottom boundaries
                if (std::abs(p2.x - p1.x) > 1e-9) {
                    allSegments.push_back({p1, p2});
                }
            }
        };

        // Load geometry
        processPolygon(map.borders.get_points());
        for (const auto& obs : map.obstacles.get_obstacles()) {
            processPolygon(obs.get_points());
        }

        std::vector<double> sortedX(x_events.begin(), x_events.end());

        // Iterate through vertical slabs
        for (size_t i = 0; i < sortedX.size() - 1; ++i) {
            double x_start = sortedX[i];
            double x_end = sortedX[i+1];
            double x_mid = (x_start + x_end) / 2.0;

            if (x_end - x_start < 1e-6) continue; // Skip tiny slices

            // Find segments that span across this slab
            struct ActiveSegment {
                double y_mid;
                const Segment* seg;
            };
            std::vector<ActiveSegment> active;

            for (const auto& seg : allSegments) {
                // Check if segment spans the full width of [x_start, x_end]
                // Using epsilon for robust floating point comparison
                if (seg.p1.x <= x_start + 1e-7 && seg.p2.x >= x_end - 1e-7) {
                    active.push_back({seg.getYAtX(x_mid), &seg});
                }
            }

            // Sort segments by Y (bottom to top)
            std::sort(active.begin(), active.end(), [](const ActiveSegment& a, const ActiveSegment& b) {
                return a.y_mid < b.y_mid;
            });

            // Create trapezoids between consecutive segments
            for (size_t k = 0; k < active.size() - 1; ++k) {
                const Segment* botSeg = active[k].seg;
                const Segment* topSeg = active[k+1].seg;

                double y_mid_check = (active[k].y_mid + active[k+1].y_mid) / 2.0;
                Vertex probe(x_mid, y_mid_check);

                // Verify this region is free space
                if (isFreeSpace(probe, map)) {
                    double tly = topSeg->getYAtX(x_start);
                    double try_ = topSeg->getYAtX(x_end);
                    double bly = botSeg->getYAtX(x_start);
                    double bry = botSeg->getYAtX(x_end);

                    // Construct your Trapezoid
                    Trapezoid trap(x_start, x_end, tly, try_, bly, bry);
                    result.push_back(trap);
                }
            }
        }
        return result;
    }

    void connectAdjacentTrapezoids(std::vector<Trapezoid>& trapezoids) {
        // Simple O(N^2) connectivity check. 
        // Can be optimized to O(N) by grouping trapezoids by their x_left/x_right values.
        
        for (size_t i = 0; i < trapezoids.size(); ++i) {
            for (size_t j = i + 1; j < trapezoids.size(); ++j) {
                Trapezoid& t1 = trapezoids[i];
                Trapezoid& t2 = trapezoids[j];

                // Check if they share a vertical wall
                bool rightToLeft = std::abs(t1.rightX - t2.leftX) < 1e-6;
                bool leftToRight = std::abs(t1.leftX - t2.rightX) < 1e-6;

                if (!rightToLeft && !leftToRight) continue;

                // Determine the shared X coordinate
                double sharedX = rightToLeft ? t1.rightX : t1.leftX;

                // Get the Y intervals at the shared boundary
                // t1 interval at sharedX
                double t1_y_high = rightToLeft ? t1.topRightY : t1.topLeftY;
                double t1_y_low  = rightToLeft ? t1.bottomRightY : t1.bottomLeftY;

                // t2 interval at sharedX
                double t2_y_high = rightToLeft ? t2.topLeftY : t2.topRightY;
                double t2_y_low  = rightToLeft ? t2.bottomLeftY : t2.bottomRightY;

                // Check interval overlap
                double overlapStart = std::max(t1_y_low, t2_y_low);
                double overlapEnd   = std::min(t1_y_high, t2_y_high);

                if (overlapEnd > overlapStart + 1e-6) {
                    t1.neighbors.push_back(j);
                    t2.neighbors.push_back(i);
                }
            }
        }
    }
}