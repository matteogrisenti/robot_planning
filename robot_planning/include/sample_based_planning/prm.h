#ifndef PROBABILISTIC_ROADMAP_H
#define PROBABILISTIC_ROADMAP_H

#include <vector>
#include <memory>
#include <random>

#include "map/map_data_structures.h"
#include "roadmap/roadmap_data_structures.h"
// Includiamo le tue librerie esistenti
#include "dubins_planner/collision_checker.h" 
#include "dubins_planner/dubins_trajectory.h"

namespace SampleBasedPlanning {

class PRMPlanner {
public:
    PRMPlanner(int num_samples, double connection_radius, double rho, double robot_radius);

    std::shared_ptr<Roadmap> buildRoadmap(const Map& map);

private:
    int num_samples_;
    double connection_radius_;
    double rho_; // Raggio minimo di sterzata
    double robot_radius_;

    std::mt19937 rng_;

    void addPointsOfInterest(Roadmap& roadmap, const Map& map, const CollisionChecker& checker);
    void sampleFreeSpace(Roadmap& roadmap, const Map& map, const CollisionChecker& checker);
    void connectNodes(Roadmap& roadmap, const CollisionChecker& checker);
};

} 
#endif