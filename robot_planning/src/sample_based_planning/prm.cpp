#include "sample_based_planning/prm.h"
#include "combinatorial_planning/planning_utils.h"
#include <random>
#include <iostream>
#include <ros/ros.h>

namespace sample_planning {

    // Helper: Check if a point is valid (Inside borders, Outside obstacles)
    bool isConfigurationFree(const Vertex& p, const Map& map) {
        // 1. Check Borders (Must be inside)
        std::vector<Vertex> borderPoly;
        for(const auto& bp : map.borders.get_points()) {
            borderPoly.push_back(Vertex(bp.x, bp.y));
        }
        if (!PlanningUtils::pointInPolygon(p, borderPoly)) return false;

        // 2. Check Obstacles (Must be outside)
        if (PlanningUtils::pointInAnyObstacle(p, map.obstacles.get_obstacles())) return false;

        return true;
    }

    std::shared_ptr<Roadmap> buildPRM(const Map& map, const PRMConfig& config) {
        auto roadmap = std::make_shared<Roadmap>();
        roadmap->setMap(&map);

        ROS_INFO("[PRM] Starting construction with N=%d, K=%d", config.num_samples, config.k_neighbors);

        // --- PHASE 1: SAMPLING ---
        // Slide lines 1-6
        
        // 1. Determine Sampling Bounds
        double minX, minY, maxX, maxY;
        map.get_bounding_box(minX, minY, maxX, maxY);
        
        // Random Number Generation setup
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> disX(minX, maxX);
        std::uniform_real_distribution<> disY(minY, maxY);

        int samples_added = 0;
        int max_attempts = config.num_samples * 100; // Safety break
        int attempts = 0;

        while (samples_added < config.num_samples && attempts < max_attempts) {
            attempts++;
            
            // 2. Generate Sample (q_rand)
            Vertex q(disX(gen), disY(gen));

            // 3. Check Validity (CLEAR(q))
            if (isConfigurationFree(q, map)) {
                roadmap->addVertex(q);
                samples_added++;
            }
        }

        if (samples_added < config.num_samples) {
            ROS_WARN("[PRM] Could only sample %d/%d valid points within bounding box.", 
                     samples_added, config.num_samples);
        }

        // --- PHASE 2: CONNECTING ---
        // Slide lines 7-17

        int num_vertices = roadmap->getNumVertices();
        int edges_added = 0;

        for (int i = 0; i < num_vertices; ++i) {
            const Vertex& q = roadmap->getVertex(i);

            // 1. Find Neighbors (KNEAR)
            // Using the existing helper in Roadmap
            std::vector<int> neighbors = roadmap->findKNearestNeighbors(i, config.k_neighbors);

            for (int neighbor_idx : neighbors) {
                // Avoid self-loops and duplicate checks (check only if i < neighbor_idx for undirected)
                if (i >= neighbor_idx) continue;

                const Vertex& q_near = roadmap->getVertex(neighbor_idx);

                // Optional: Check max connection distance (not strictly in slide, but good practice)
                if (config.max_connection_dist > 0 && q.distance(q_near) > config.max_connection_dist) {
                    continue;
                }

                // 2. Local Planner / Collision Check (PATH & COLLISION)
                // We assume a straight line local planner.
                // Using PlanningUtils to check line segment collision.
                if (!PlanningUtils::lineSegmentIntersectsObstacle(q, q_near, map.obstacles.get_obstacles())) {
                    
                    // 3. Add Edge
                    // Weight is Euclidean distance (calculated automatically by addEdge overload #1)
                    roadmap->addEdge(i, neighbor_idx, true);
                    edges_added++;
                }
            }
        }

        ROS_INFO("[PRM] Construction complete. Vertices: %d, Edges: %d", samples_added, edges_added);
        return roadmap;
    }

}