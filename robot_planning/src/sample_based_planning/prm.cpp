#include "sample_based_planning/prm.h"
#include "combinatorial_planning/planning_utils.h" // NECESSARIO per pointInPolygon
#include <iostream>
#include <cmath>
#include <ros/ros.h>

namespace SampleBasedPlanning {

PRMPlanner::PRMPlanner(int num_samples, double connection_radius, double rho, double robot_radius)
    : num_samples_(num_samples), connection_radius_(connection_radius), rho_(rho), robot_radius_(robot_radius) 
{
    std::random_device rd;
    rng_ = std::mt19937(rd());
}

std::shared_ptr<Roadmap> PRMPlanner::buildRoadmap(const Map& map) {
    auto roadmap = std::make_shared<Roadmap>();
    roadmap->setMap(&map);

    // 1. Setup Collision Checker
    CollisionChecker checker(robot_radius_);
    checker.update_collision_cache(map);

    // 2. Aggiungi Start, Gate, Vittime
    addPointsOfInterest(*roadmap, map, checker);

    // 3. Campiona (Ora con controllo Bordi!)
    sampleFreeSpace(*roadmap, map, checker);

    // 4. Connetti usando Dubins
    connectNodes(*roadmap, checker);

    return roadmap;
}

void PRMPlanner::addPointsOfInterest(Roadmap& roadmap, const Map& map, const CollisionChecker& checker) {
    // START
    const auto& s = map.start;
    double theta_start = 2.0 * std::atan2(s.get_orientation().z, s.get_orientation().w);
    
    if (checker.is_state_valid(s.get_position().x, s.get_position().y)) {
        roadmap.addVertex(Vertex(s.get_position().x, s.get_position().y, theta_start));
    }

    // VITTIME (4 orientamenti)
    for (const auto& v : map.victims.get_victims()) {
        if (checker.is_state_valid(v.get_center().x, v.get_center().y)) {
            for (double th = 0; th < 2*M_PI; th += M_PI/2.0) {
                roadmap.addVertex(Vertex(v.get_center().x, v.get_center().y, th));
            }
        }
    }
    
    // GATE
    for (const auto& g : map.gates.get_gates()) {
        double theta_gate = 2.0 * std::atan2(g.get_orientation().z, g.get_orientation().w);
        if (checker.is_state_valid(g.get_position().x, g.get_position().y)) {
            roadmap.addVertex(Vertex(g.get_position().x, g.get_position().y, theta_gate));
        }
    }
}

void PRMPlanner::sampleFreeSpace(Roadmap& roadmap, const Map& map, const CollisionChecker& checker) {
    double minX, minY, maxX, maxY;
    map.get_bounding_box(minX, minY, maxX, maxY);
    
    // Convertiamo i bordi della mappa in un formato usabile da PlanningUtils
    std::vector<Vertex> border_polygon;
    for(const auto& p : map.borders.get_points()) {
        border_polygon.emplace_back(p.x, p.y);
    }

    std::uniform_real_distribution<double> dist_x(minX, maxX);
    std::uniform_real_distribution<double> dist_y(minY, maxY);
    std::uniform_real_distribution<double> dist_th(-M_PI, M_PI);

    int count = 0;
    int attempts = 0;
    // Timeout di sicurezza per evitare loop infiniti se la mappa è piena
    int max_attempts = num_samples_ * 200; 

    while(count < num_samples_ && attempts < max_attempts) {
        attempts++;
        double x = dist_x(rng_);
        double y = dist_y(rng_);
        Vertex v_sample(x, y);

        // CHECK 1: Il punto deve essere DENTRO i bordi della mappa (Esagono)
        if (!PlanningUtils::pointInPolygon(v_sample, border_polygon)) {
            continue; 
        }

        // CHECK 2: Il punto deve essere LONTANO dagli ostacoli
        if (checker.is_state_valid(x, y)) {
            roadmap.addVertex(Vertex(x, y, dist_th(rng_)));
            count++;
        }
    }
    
    if (count < num_samples_) {
        ROS_WARN("[PRM] Generated only %d/%d samples. Map might be crowded.", count, num_samples_);
    }
}

void PRMPlanner::connectNodes(Roadmap& roadmap, const CollisionChecker& checker) {
    int n = roadmap.getNumVertices();
    long double k_max = (long double)(1.0 / rho_);

    for (int i = 0; i < n; ++i) {
        const Vertex& v1 = roadmap.getVertex(i);
        auto neighbors = roadmap.getNearestNeighbors(i, connection_radius_);

        for (int j : neighbors) {
            const Vertex& v2 = roadmap.getVertex(j);

            dubinscurve_out curve;
            int best_word = -1;

            dubins_shortest_path(
                (long double)v1.x, (long double)v1.y, (long double)v1.theta,
                (long double)v2.x, (long double)v2.y, (long double)v2.theta,
                k_max, best_word, &curve
            );

            if (best_word >= 0) {
                // Controllo collisioni accurato lungo la curva
                if (checker.is_dubins_path_valid(best_word, curve)) {
                    // Nota: Anche se l'arco è valido, il visualizzatore disegnerà una retta.
                    // Se la retta passa sull'ostacolo ma la curva no, è normale.
                    roadmap.addEdge(i, j, (double)curve.L, false, best_word);
                }
            }
        }
    }
}

} // namespace