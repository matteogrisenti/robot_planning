#include "libraries/roadmap_factory.h"
#include "libraries/combinatorial_planning.h"
#include "libraries/sample_based_planning.h"
#include <ros/ros.h>

std::shared_ptr<Roadmap> generateRoadmap(const std::string& type, const Map& map) {
    if (type == "prm") {
        sample_planning::PRMConfig config;
        config.num_samples = 1500; 
        config.k_neighbors = 15;
        return sample_planning::buildPRM(map, config);
    } 
    else if (type == "ecd") return exactCellDecomposition(map);
    else if (type == "acd") return approximateCellDecomposition(map, 4);
    else if (type == "mcr") return maximumClearanceRoadmap(map);
    else if (type == "spr") return shortestPathRoadmap(map, 0.5);
    else if (type == "rrt") {
        sample_planning::RRTConfig config; 
        config.max_iterations = 2000; 
        config.step_size = 0.5;
        return sample_planning::buildRRT(map, config);
    }
    else if (type == "rrt_star") {
        sample_planning::RRTStarConfig config; 
        config.max_iterations = 2000; 
        config.step_size = 0.4;
        config.search_radius = 1.2;
        return sample_planning::buildRRTStar(map, config);
    }
    else {
        ROS_ERROR("Planner Type '%s' NOT RECOGNIZED!", type.c_str());
        return nullptr;
    }
}