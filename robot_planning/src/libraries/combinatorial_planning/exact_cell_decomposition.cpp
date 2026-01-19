#include "libraries/combinatorial_planning/exact_cell_decomposition.h"
#include "libraries/planning_utils.h" 
#include <algorithm>
#include <set>
#include <limits>
#include <iostream>
#include <ros/ros.h>

namespace HelperExactDecomposition {
    struct Segment; 
    std::vector<Trapezoid> computeTrapezoidalDecomposition(const Map& map, const std::vector<Point>& extraPoints);
    void connectAdjacentTrapezoids(std::vector<Trapezoid>& trapezoids);
}

// --- Main Function ---
std::shared_ptr<Roadmap> exactCellDecomposition(const Map& map) {
    auto roadmap = std::make_shared<Roadmap>();
    roadmap->setMap(&map);

    // --- 0. PREPARAZIONE TARGETS ---
    std::vector<Point> targets;
    if (!map.gates.get_gates().empty()) {
        targets.push_back(map.gates.get_gates()[0].get_position());
    }
    for (const auto& v : map.victims.get_victims()) {
        targets.push_back(v.get_center());
    }

    // --- 1. Compute Decomposition ---
    std::vector<Trapezoid> trapezoids = HelperExactDecomposition::computeTrapezoidalDecomposition(map, targets);
    roadmap->debugTrapezoids = std::make_shared<std::vector<Trapezoid>>(trapezoids);

    // 2. Compute Adjacency
    HelperExactDecomposition::connectAdjacentTrapezoids(trapezoids);

    // 3. Build Roadmap Graph
    std::vector<int> trapIdToNodeId(trapezoids.size());
    for (size_t i = 0; i < trapezoids.size(); ++i) {
        trapIdToNodeId[i] = roadmap->addVertex(trapezoids[i].center);
    }

    // Collega i target
    for (const auto& p : targets) {
        Vertex t(p.x, p.y);
        int tNodeIdx = roadmap->addVertex(t);
        bool connected = false;

        for(size_t i = 0; i < trapezoids.size(); ++i) {
            const auto& tr = trapezoids[i];
            double eps = 1e-3;
            if (t.x >= tr.leftX - eps && t.x <= tr.rightX + eps) {
                double minY = std::min(tr.bottomLeftY, tr.bottomRightY);
                double maxY = std::max(tr.topLeftY, tr.topRightY);
                if (t.y >= minY - eps && t.y <= maxY + eps) {
                    roadmap->addEdge(tNodeIdx, trapIdToNodeId[i], true);
                    connected = true;
                }
            }
        }
        if (!connected) {
             int nearest = -1; 
             double minDist = std::numeric_limits<double>::max();
             for(size_t i=0; i<trapezoids.size(); ++i) {
                 double d = trapezoids[i].center.distance(t);
                 if(d < minDist) { minDist=d; nearest=trapIdToNodeId[i]; }
             }
             if(nearest != -1) roadmap->addEdge(tNodeIdx, nearest, true);
        }
    }

    // --- DEFINIZIONE VARIABILE MANCANTE ---
    double min_passage_width = 0.70; 

    // 4. Connect Trapezoids via Gateways
    for (size_t i = 0; i < trapezoids.size(); ++i) {
        const auto& t1 = trapezoids[i];
        for (int neighborIdx : trapezoids[i].neighbors) {
            if (neighborIdx <= (int)i) continue;

            const auto& t2 = trapezoids[neighborIdx];
            bool t1IsLeft = std::abs(t1.rightX - t2.leftX) < 1e-6;
            double sharedX = t1IsLeft ? t1.rightX : t1.leftX;
            double t1_y_high = t1IsLeft ? t1.topRightY : t1.topLeftY;
            double t1_y_low  = t1IsLeft ? t1.bottomRightY : t1.bottomLeftY;
            double t2_y_high = t1IsLeft ? t2.topLeftY : t2.topRightY;
            double t2_y_low  = t1IsLeft ? t2.bottomLeftY : t2.bottomRightY;
            double overlapStart = std::max(t1_y_low, t2_y_low);
            double overlapEnd   = std::min(t1_y_high, t2_y_high);
            
            double passage_width = overlapEnd - overlapStart;

            // CHECK LARGHEZZA PASSAGGIO
            if (passage_width < min_passage_width) {
                continue; 
            }

            Vertex gateway(sharedX, (overlapStart + overlapEnd) / 2.0);
            if (!PlanningUtils::isPointValid(gateway.x, gateway.y, map, 0.5)) continue;
            
            int gatewayNodeId = roadmap->addVertex(gateway);
            roadmap->addEdge(trapIdToNodeId[i], gatewayNodeId, true);
            roadmap->addEdge(gatewayNodeId, trapIdToNodeId[neighborIdx], true);
        }
    }

    return roadmap;
}

// ... (Il resto del namespace HelperExactDecomposition rimane uguale) ...
namespace HelperExactDecomposition {
    struct Segment {
        Point p1, p2;
        double getYAtX(double x) const {
            if (std::abs(p2.x - p1.x) < 1e-9) return std::max(p1.y, p2.y); 
            double t = (x - p1.x) / (p2.x - p1.x);
            return p1.y + t * (p2.y - p1.y);
        }
    };

    bool isFreeSpace(const Vertex& p, const Map& map) {
        std::vector<Vertex> borderPoly;
        for(const auto& bp : map.borders.get_points()) borderPoly.push_back(Vertex(bp.x, bp.y));
        if (!PlanningUtils::pointInPolygon(p, borderPoly)) return false;
        if (PlanningUtils::pointInAnyObstacle(p, map.obstacles.get_obstacles())) return false;
        return true;
    }
    
    std::vector<Trapezoid> computeTrapezoidalDecomposition(const Map& map, const std::vector<Point>& extraPoints) {
        std::vector<Trapezoid> result;
        std::vector<Segment> allSegments;
        std::set<double> x_events;

        auto processPolygon = [&](const std::vector<Point>& pts) {
            if (pts.empty()) return;
            for (size_t i = 0; i < pts.size(); ++i) {
                Point p1 = pts[i];
                Point p2 = pts[(i + 1) % pts.size()];
                x_events.insert(p1.x); 
                if (p1.x > p2.x) std::swap(p1, p2);
                if (std::abs(p2.x - p1.x) > 1e-9) {
                    allSegments.push_back({p1, p2});
                }
            }
        };

        processPolygon(map.borders.get_points());
        for (const auto& obs : map.obstacles.get_obstacles()) {
            processPolygon(obs.get_points());
        }

        for (const auto& p : extraPoints) {
            x_events.insert(p.x);
        }

        std::vector<double> sortedX(x_events.begin(), x_events.end());

        for (size_t i = 0; i < sortedX.size() - 1; ++i) {
            double x_start = sortedX[i];
            double x_end = sortedX[i+1];
            double x_mid = (x_start + x_end) / 2.0;

            if (x_end - x_start < 1e-6) continue;

            struct ActiveSegment {
                double y_mid;
                const Segment* seg;
            };
            std::vector<ActiveSegment> active;

            for (const auto& seg : allSegments) {
                if (seg.p1.x <= x_start + 1e-7 && seg.p2.x >= x_end - 1e-7) {
                    active.push_back({seg.getYAtX(x_mid), &seg});
                }
            }

            std::sort(active.begin(), active.end(), [](const ActiveSegment& a, const ActiveSegment& b) {
                return a.y_mid < b.y_mid;
            });

            for (size_t k = 0; k < active.size() - 1; ++k) {
                const Segment* botSeg = active[k].seg;
                const Segment* topSeg = active[k+1].seg;
                double y_mid_check = (active[k].y_mid + active[k+1].y_mid) / 2.0;
                Vertex probe(x_mid, y_mid_check);

                if (isFreeSpace(probe, map)) {
                    double tly = topSeg->getYAtX(x_start);
                    double try_ = topSeg->getYAtX(x_end);
                    double bly = botSeg->getYAtX(x_start);
                    double bry = botSeg->getYAtX(x_end);
                    result.push_back(Trapezoid(x_start, x_end, tly, try_, bly, bry));
                }
            }
        }
        return result;
    }

    void connectAdjacentTrapezoids(std::vector<Trapezoid>& trapezoids) {
        for (size_t i = 0; i < trapezoids.size(); ++i) {
            for (size_t j = i + 1; j < trapezoids.size(); ++j) {
                Trapezoid& t1 = trapezoids[i];
                Trapezoid& t2 = trapezoids[j];

                bool rightToLeft = std::abs(t1.rightX - t2.leftX) < 1e-6;
                bool leftToRight = std::abs(t1.leftX - t2.rightX) < 1e-6;

                if (!rightToLeft && !leftToRight) continue;

                double t1_y_high = rightToLeft ? t1.topRightY : t1.topLeftY;
                double t1_y_low  = rightToLeft ? t1.bottomRightY : t1.bottomLeftY;
                double t2_y_high = rightToLeft ? t2.topLeftY : t2.topRightY;
                double t2_y_low  = rightToLeft ? t2.bottomLeftY : t2.bottomRightY;

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