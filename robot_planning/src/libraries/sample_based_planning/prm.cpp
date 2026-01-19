#include "libraries/sample_based_planning/prm.h"
#include "libraries/planning_utils.h"
#include <random>
#include <iostream>
#include <ros/ros.h>

namespace sample_planning {

    bool isConfigurationFree(const Vertex& p, const Map& map) {
            return PlanningUtils::isPointValid(p.x, p.y, map, 0.5);
        }

    std::shared_ptr<Roadmap> buildPRM(const Map& map, const PRMConfig& config) {
        auto roadmap = std::make_shared<Roadmap>();
        roadmap->setMap(&map);

        ROS_INFO("[PRM] Starting construction with N=%d, K=%d", config.num_samples, config.k_neighbors);

        // --- PHASE 1: SAMPLING ---
        
        // 1. Determine Sampling Bounds
        double minX, minY, maxX, maxY;
        map.get_bounding_box(minX, minY, maxX, maxY);
        
        // Random Number Generation setup
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> disX(minX, maxX);
        std::uniform_real_distribution<> disY(minY, maxY);

        int samples_added = 0;

        // ==================================================================================
        // NEW: INIEZIONE ESPLICITA TARGET (Start, Gate, Vittime)
        // Inseriamo questi nodi PRIMA del random sampling per garantire che esistano nel grafo
        // ==================================================================================
        
        // 1. Start
        roadmap->addVertex(Vertex(map.start.get_position().x, map.start.get_position().y));
        samples_added++;

        // 2. Gate (se presente)
        if (!map.gates.get_gates().empty()) {
            Point g = map.gates.get_gates()[0].get_position();
            // Controlliamo la validità per sicurezza, ma li aggiungiamo prioritariamente
            Vertex gateV(g.x, g.y);
            if (isConfigurationFree(gateV, map)) {
                roadmap->addVertex(gateV);
                samples_added++;
            } else {
                ROS_WARN("[PRM] Gate position is in collision or out of bounds! Not added.");
            }
        }

        // 3. Vittime
        for (const auto& v : map.victims.get_victims()) {
            Point p = v.get_center();
            Vertex victimV(p.x, p.y);
            
            // Nota: Le vittime sono spesso circondate da spazio libero per definizione, 
            // ma un check veloce non fa male.
            if (isConfigurationFree(victimV, map)) {
                roadmap->addVertex(victimV);
                samples_added++;
            } else {
                ROS_WARN("[PRM] Victim at (%.2f, %.2f) is invalid! Not added.", p.x, p.y);
            }
        }
        
        ROS_INFO("[PRM] Injected %d critical nodes (Start/Gate/Victims). Filling rest with random samples...", samples_added);
        // ==================================================================================


        int max_attempts = config.num_samples * 100; // Safety break
        int attempts = 0;

        // Riempiamo il resto del grafo fino a N campioni
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

                // Optional: Check max connection distance
                if (config.max_connection_dist > 0 && q.distance(q_near) > config.max_connection_dist) {
                    continue;
                }

                // 2. Local Planner / Collision Check (PATH & COLLISION)
                if (!PlanningUtils::lineSegmentIntersectsObstacle(q, q_near, map.obstacles.get_obstacles())) {
                    
                    // 3. Add Edge
                    roadmap->addEdge(i, neighbor_idx, true);
                    edges_added++;
                }
            }
        }

        ROS_INFO("[PRM] Construction complete. Vertices: %d, Edges: %d", samples_added, edges_added);
        return roadmap;
    }

}