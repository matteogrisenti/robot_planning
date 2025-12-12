#include "sample_based_planning/rrt_star.h"
#include "combinatorial_planning/planning_utils.h"
#include <random>
#include <cmath>
#include <limits>
#include <iostream>
#include <ros/ros.h>

namespace sample_planning {

    // Helper: Steer
    Vertex steer_s(const Vertex& from, const Vertex& to, double step_size) {
        double dist = from.distance(to);
        if (dist <= step_size) return to;
        double theta = std::atan2(to.y - from.y, to.x - from.x);
        return Vertex(
            from.x + step_size * std::cos(theta),
            from.y + step_size * std::sin(theta)
        );
    }

    struct NodeData {
        double cost; 
        int parent;  
    };

    std::shared_ptr<Roadmap> buildRRTStar(const Map& map, const RRTStarConfig& config) {
        auto roadmap = std::make_shared<Roadmap>();
        roadmap->setMap(&map);
        
        std::vector<NodeData> tree_data;

        // 1. Init
        Vertex startNode(map.start.get_position().x, map.start.get_position().y);
        roadmap->addVertex(startNode);
        tree_data.push_back({0.0, -1});

        // Setup RNG & Cache
        double minX, minY, maxX, maxY;
        map.get_bounding_box(minX, minY, maxX, maxY);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> disX(minX, maxX);
        std::uniform_real_distribution<> disY(minY, maxY);
        std::uniform_real_distribution<> disBias(0.0, 1.0);

        const auto& obstacles = map.obstacles.get_obstacles();
        std::vector<Vertex> borderPoly;
        for(const auto& bp : map.borders.get_points()) borderPoly.push_back(Vertex(bp.x, bp.y));

        for (int k = 0; k < config.max_iterations; ++k) {
            
            // 2. Sampling
            Vertex q_rand;
            if (config.stop_at_goal && disBias(gen) < config.goal_bias) {
                q_rand = config.goal_point;
            } else {
                q_rand = Vertex(disX(gen), disY(gen));
            }

            // Nearest
            int q_near_idx = -1;
            double min_dist = std::numeric_limits<double>::max();
            for (int i = 0; i < roadmap->getNumVertices(); ++i) {
                double d = roadmap->getVertex(i).distance(q_rand);
                if (d < min_dist) { min_dist = d; q_near_idx = i; }
            }

            Vertex q_near = roadmap->getVertex(q_near_idx);
            Vertex q_new = steer_s(q_near, q_rand, config.step_size);

            // 3. Check Validity
            if (!PlanningUtils::pointInPolygon(q_new, borderPoly)) continue;
            if (PlanningUtils::pointInAnyObstacle(q_new, obstacles)) continue;
            if (PlanningUtils::lineSegmentIntersectsObstacle(q_near, q_new, obstacles)) continue;

            // --- RRT* START ---

            // A. Find Neighbors
            std::vector<int> X_near;
            for (int i = 0; i < roadmap->getNumVertices(); ++i) {
                if (roadmap->getVertex(i).distance(q_new) <= config.search_radius) {
                    X_near.push_back(i);
                }
            }

            // B. Parent Selection (Choose Best Parent)
            int q_min_idx = q_near_idx;
            double c_min = tree_data[q_near_idx].cost + q_near.distance(q_new);

            for (int x_near_idx : X_near) {
                const Vertex& x_near = roadmap->getVertex(x_near_idx);
                double c_potential = tree_data[x_near_idx].cost + x_near.distance(q_new);
                
                if (c_potential < c_min) {
                    if (!PlanningUtils::lineSegmentIntersectsObstacle(x_near, q_new, obstacles)) {
                        c_min = c_potential;
                        q_min_idx = x_near_idx;
                    }
                }
            }

            // Add Node to Graph
            int q_new_idx = roadmap->addVertex(q_new);
            tree_data.push_back({c_min, q_min_idx});
            roadmap->addEdge(q_min_idx, q_new_idx, true);

            // C. Rewiring
            for (int x_near_idx : X_near) {
                if (x_near_idx == q_min_idx) continue; // Skip parent

                const Vertex& x_near = roadmap->getVertex(x_near_idx);
                double new_cost_via_q_new = tree_data[q_new_idx].cost + q_new.distance(x_near);

                // Se passando per q_new il costo diminuisce...
                if (new_cost_via_q_new < tree_data[x_near_idx].cost) {
                     if (!PlanningUtils::lineSegmentIntersectsObstacle(q_new, x_near, obstacles)) {
                         
                         // 1. RIMUOVI L'ARCO col vecchio genitore
                         int old_parent = tree_data[x_near_idx].parent;
                         if (old_parent != -1) {
                             roadmap->removeEdge(old_parent, x_near_idx, true);
                         }

                         // 2. AGGIUNGI L'ARCO col nuovo genitore (q_new)
                         roadmap->addEdge(q_new_idx, x_near_idx, true);
                         
                         // 3. Aggiorna i dati interni
                         tree_data[x_near_idx].parent = q_new_idx;
                         tree_data[x_near_idx].cost = new_cost_via_q_new;
                     }
                }
            }
            // --- RRT* END ---

            if (config.stop_at_goal && q_new.distance(config.goal_point) < config.goal_tolerance) {
                ROS_INFO("[RRT*] Goal reached at iter %d", k);
                break;
            }
        }
        
        ROS_INFO("[RRT*] Built optimized tree with %d nodes", roadmap->getNumVertices());
        return roadmap;
    }
}