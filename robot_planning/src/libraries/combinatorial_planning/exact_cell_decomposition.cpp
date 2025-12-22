#include "libraries/combinatorial_planning/exact_cell_decomposition.h"
#include "planning_utils.h" 
#include <algorithm>
#include <set>
#include <limits>
#include <iostream>
#include <ros/ros.h>

// --- Main Function ---
std::shared_ptr<Roadmap> exactCellDecomposition(const Map& map) {
    auto roadmap = std::make_shared<Roadmap>();
    roadmap->setMap(&map);

    // 1. Compute Decomposition
    std::vector<Trapezoid> trapezoids = HelperExactDecomposition::computeTrapezoidalDecomposition(map);
    roadmap->debugTrapezoids = std::make_shared<std::vector<Trapezoid>>(trapezoids);

    // 2. Compute Adjacency (Neighbors)
    HelperExactDecomposition::connectAdjacentTrapezoids(trapezoids);

    // 3. Build Roadmap Nodes
    std::vector<int> trapIdToNodeId(trapezoids.size());
    
    // Aggiungi centroidi
    for (size_t i = 0; i < trapezoids.size(); ++i) {
        trapIdToNodeId[i] = roadmap->addVertex(trapezoids[i].center);
    }

    // --- INTEGRAZIONE TARGET ---
    std::vector<Vertex> targets;
    if (!map.gates.get_gates().empty()) {
        targets.push_back(Vertex(map.gates.get_gates()[0].get_position().x, map.gates.get_gates()[0].get_position().y));
    }
    for (const auto& v : map.victims.get_victims()) {
        targets.push_back(Vertex(v.get_center().x, v.get_center().y));
    }

    // Aggiungi i nodi Target e collegali al trapezio di appartenenza
    for (const auto& t : targets) {
        int tNodeIdx = roadmap->addVertex(t);
        bool connected = false;

        // Cerca il trapezio che contiene il target
        for(size_t i = 0; i < trapezoids.size(); ++i) {
            const auto& tr = trapezoids[i];
            
            // Check bounding box semplice (sufficiente per trapezi verticali della decomposizione)
            // Assunzione: Decomposizione Trapezoidale standard (linee verticali)
            // L'implementazione helper usa segmenti verticali, quindi leftX e rightX sono i confini X.
            // I confini Y sono segmenti inclinati, ma qui approssimiamo check o usiamo centroid logic.
            
            // Verifica precisa X
            if (t.x >= tr.leftX && t.x <= tr.rightX) {
                // Verifica approssimata Y (bounding box Y)
                double minY = std::min(tr.bottomLeftY, tr.bottomRightY);
                double maxY = std::max(tr.topLeftY, tr.topRightY);
                
                if (t.y >= minY && t.y <= maxY) {
                    // Trovato trapezio contenitore: collega al centroide
                    roadmap->addEdge(tNodeIdx, trapIdToNodeId[i], true);
                    connected = true;
                    // Opzionale: break se vogliamo un solo trapezio (solitamente unico)
                    break;
                }
            }
        }

        // Fallback se non cade perfettamente in un trapezio (es. sul bordo)
        if (!connected) {
             int nearest = -1;
             double minDist = std::numeric_limits<double>::max();
             // Cerca il centroide più vicino
             for(size_t i=0; i<trapezoids.size(); ++i) {
                 double d = trapezoids[i].center.distance(t);
                 if(d < minDist) { minDist=d; nearest=trapIdToNodeId[i]; }
             }
             if(nearest != -1) roadmap->addEdge(tNodeIdx, nearest, true);
        }
    }

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

            Vertex gateway(sharedX, (overlapStart + overlapEnd) / 2.0);
            int gatewayNodeId = roadmap->addVertex(gateway);

            // Collega il gateway ai centroidi
            roadmap->addEdge(trapIdToNodeId[i], gatewayNodeId, true);
            roadmap->addEdge(gatewayNodeId, trapIdToNodeId[neighborIdx], true);
        }
    }

    return roadmap;
}

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
    
    std::vector<Trapezoid> computeTrapezoidalDecomposition(const Map& map) {
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