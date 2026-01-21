#include "libraries/sample_based_planning/rrt.h"
#include "libraries/planning_utils.h"
#include <random>
#include <cmath>
#include <limits>
#include <iostream>
#include <ros/ros.h>

namespace sample_planning {

    // Helper: Nearest Neighbor
    int getNearestNeighborIdx(const Roadmap& r, const Vertex& q_rand) {
        int nearestIdx = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (int i = 0; i < r.getNumVertices(); ++i) {
            double d = r.getVertex(i).distance(q_rand);
            if (d < min_dist) {
                min_dist = d;
                nearestIdx = i;
            }
        }
        return nearestIdx;
    }

    // Helper: Steer
    Vertex steer(const Vertex& from, const Vertex& to, double step_size) {
        double dist = from.distance(to);
        if (dist <= step_size) {
            return to; 
        } else {
            double theta = std::atan2(to.y - from.y, to.x - from.x);
            return Vertex(
                from.x + step_size * std::cos(theta),
                from.y + step_size * std::sin(theta)
            );
        }
    }

    // Helper: Final Connection
    void connectTargetToTree(Roadmap& roadmap, const Vertex& target, const std::vector<Obstacle>& obstacles) {
        int nearestIdx = getNearestNeighborIdx(roadmap, target);
        if (nearestIdx == -1) return;

        const Vertex& nearestNode = roadmap.getVertex(nearestIdx);
        if (!PlanningUtils::lineSegmentIntersectsObstacle(nearestNode, target, obstacles)) {
            int targetIdx = roadmap.addVertex(target);
            roadmap.addEdge(nearestIdx, targetIdx, true); // Bidirectional
        }
    }

    std::shared_ptr<Roadmap> buildRRT(const Map& map, const RRTConfig& config) {
        auto roadmap = std::make_shared<Roadmap>();
        roadmap->setMap(&map);

        // Root (Start)
        Vertex startNode(map.start.get_position().x, map.start.get_position().y);
        roadmap->addVertex(startNode);

        // Setup RNG
        double minX, minY, maxX, maxY;
        map.get_bounding_box(minX, minY, maxX, maxY);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> disX(minX, maxX);
        std::uniform_real_distribution<> disY(minY, maxY);
        std::uniform_real_distribution<> disBias(0.0, 1.0);

        // Cache Obstacles and Borders
        const auto& obstacles = map.obstacles.get_obstacles();
        std::vector<Vertex> borderPoly;
        for(const auto& bp : map.borders.get_points()) borderPoly.push_back(Vertex(bp.x, bp.y));

        // Goal Biasing
        std::vector<Vertex> targets;
        if (!map.gates.get_gates().empty()) {
            Point g = map.gates.get_gates()[0].get_position();
            targets.push_back(Vertex(g.x, g.y));
        }
        for (const auto& v : map.victims.get_victims()) {
            Point p = v.get_center();
            targets.push_back(Vertex(p.x, p.y));
        }
        double goal_bias_prob = 0.10; // 10% prob of pointing to a target

        // Main Loop
        for (int k = 0; k < config.max_iterations; ++k) {
            
            Vertex q_rand;

            // Goal Biasing
            if (!targets.empty() && disBias(gen) < goal_bias_prob) {
                // Choose a target at random
                int tIdx = std::rand() % targets.size();
                q_rand = targets[tIdx];
            } else {
                // Uniform sampling
                q_rand = Vertex(disX(gen), disY(gen));
               if (!PlanningUtils::isPointValid(q_rand.x, q_rand.y, map, 0.5)) {
                    continue; 
                }
            }

            // Check validity
            if (!PlanningUtils::pointInPolygon(q_rand, borderPoly)) continue;
            if (PlanningUtils::pointInAnyObstacle(q_rand, obstacles)) continue; 

            // NEAREST
            int q_near_idx = getNearestNeighborIdx(*roadmap, q_rand);
            if (q_near_idx == -1) continue;
            Vertex q_near = roadmap->getVertex(q_near_idx);

            // EXTEND
            Vertex q_new = steer(q_near, q_rand, config.step_size);

            if (!PlanningUtils::isPointValid(q_new.x, q_new.y, map, 0.5)) {
                continue;
            }
            if (PlanningUtils::lineSegmentIntersectsObstacle(q_near, q_new, obstacles)) continue;

            // ADD VERTEX & EDGE
            int q_new_idx = roadmap->addVertex(q_new);
            roadmap->addEdge(q_near_idx, q_new_idx, true); // Bidirectional
        }

        // Explicit Target Connection
        // At the end, try to connect all targets to the nearest branch
        // This ensures that A* can reach the exact coordinates of the victim
        ROS_INFO("[RRT] Tree built (%d nodes). Connecting Targets...", roadmap->getNumVertices());
        for (const auto& t : targets) {
            connectTargetToTree(*roadmap, t, obstacles);
        }

        return roadmap;
    }
}