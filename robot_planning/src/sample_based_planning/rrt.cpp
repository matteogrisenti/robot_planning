#include "sample_based_planning/rrt.h"
#include "combinatorial_planning/planning_utils.h"
#include <random>
#include <cmath>
#include <limits>
#include <iostream>
#include <ros/ros.h>

namespace sample_planning {

    // Helper: NEAREST(G, q_rand) - Slide riga 5
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

    // Helper: EXTEND(q_nearest, q_rand) - Slide riga 6
    // Calcola q_new muovendosi di 'step_size' verso q_rand
    Vertex steer(const Vertex& from, const Vertex& to, double step_size) {
        double dist = from.distance(to);
        if (dist <= step_size) {
            return to; // Raggiungibile in un passo
        } else {
            double theta = std::atan2(to.y - from.y, to.x - from.x);
            return Vertex(
                from.x + step_size * std::cos(theta),
                from.y + step_size * std::sin(theta)
            );
        }
    }

    std::shared_ptr<Roadmap> buildRRT(const Map& map, const RRTConfig& config) {
        auto roadmap = std::make_shared<Roadmap>();
        roadmap->setMap(&map);

        // 1. G.addVertex(q_init) - Slide riga 1
        Vertex startNode(map.start.get_position().x, map.start.get_position().y);
        roadmap->addVertex(startNode);

        // Setup Generazione Numeri Casuali
        double minX, minY, maxX, maxY;
        map.get_bounding_box(minX, minY, maxX, maxY);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> disX(minX, maxX);
        std::uniform_real_distribution<> disY(minY, maxY);
        std::uniform_real_distribution<> disBias(0.0, 1.0);

        // Cache per collision checking veloce
        const auto& obstacles = map.obstacles.get_obstacles();
        std::vector<Vertex> borderPoly;
        for(const auto& bp : map.borders.get_points()) borderPoly.push_back(Vertex(bp.x, bp.y));

        bool goal_reached = false;

        // 2. while q_goal not reached do - Slide riga 2
        // (Usiamo un for loop con break per evitare loop infiniti se il goal è irraggiungibile)
        for (int k = 0; k < config.max_iterations; ++k) {
            
            // Check condition "Goal Reached"
            if (config.stop_at_goal && goal_reached) break;

            // 3. q_rand = nextSample() - Slide riga 3
            Vertex q_rand;
            
            // Goal Bias: Ogni tanto puntiamo al goal invece che a caso
            if (config.stop_at_goal && disBias(gen) < config.goal_bias) {
                q_rand = config.goal_point;
            } else {
                q_rand = Vertex(disX(gen), disY(gen));
            }

            // 4. if CLEAR(q_rand) - Slide riga 4 (Opzionale in RRT standard, ma richiesto dalla slide)
            // Nota: RRT standard a volte accetta q_rand in collisione purché q_new sia libero, 
            // ma seguiamo la slide che chiede CLEAR(q_rand).
            // Controllo veloce se è nei bordi e fuori dagli ostacoli
            if (!PlanningUtils::pointInPolygon(q_rand, borderPoly)) continue;
            // Ottimizzazione: q_rand in ostacolo è spesso accettabile in RRT "vanilla", 
            // ma la slide dice "if CLEAR", quindi scartiamo.
            if (PlanningUtils::pointInAnyObstacle(q_rand, obstacles)) continue; 


            // 5. q_nearest = NEAREST(G, q_rand) - Slide riga 5
            int q_near_idx = getNearestNeighborIdx(*roadmap, q_rand);
            if (q_near_idx == -1) continue; 
            Vertex q_near = roadmap->getVertex(q_near_idx);

            // 6. (q_new, path) = EXTEND(q_nearest, q_rand) - Slide riga 6
            Vertex q_new = steer(q_near, q_rand, config.step_size);

            // 7. if not COLLISION(path) - Slide riga 7
            // Verifichiamo sia il punto q_new che il segmento [q_near, q_new]
            if (PlanningUtils::pointInAnyObstacle(q_new, obstacles)) continue;
            if (!PlanningUtils::pointInPolygon(q_new, borderPoly)) continue;
            if (PlanningUtils::lineSegmentIntersectsObstacle(q_near, q_new, obstacles)) continue;

            // 8. G.addVertex(q_new) - Slide riga 8
            int q_new_idx = roadmap->addVertex(q_new);

            // 9. G.addEdge(q_nearest, q_new, path) - Slide riga 9
            roadmap->addEdge(q_near_idx, q_new_idx, true); 

            // Check se abbiamo raggiunto il goal
            if (config.stop_at_goal) {
                if (q_new.distance(config.goal_point) <= config.goal_tolerance) {
                    ROS_INFO("[RRT] Goal Reached at iteration %d!", k);
                    goal_reached = true;
                    // Opzionale: Connetti direttamente al goal esatto se la linea è libera
                    if (!PlanningUtils::lineSegmentIntersectsObstacle(q_new, config.goal_point, obstacles)) {
                         int goal_idx = roadmap->addVertex(config.goal_point);
                         roadmap->addEdge(q_new_idx, goal_idx, true);
                    }
                }
            }
        }

        ROS_INFO("[RRT] Built tree with %d nodes", roadmap->getNumVertices());
        return roadmap;
    }
}