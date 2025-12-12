#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "sample_based_planning/rrt.h"
#include <vector>

namespace sample_planning {

    struct RRTStarConfig {
        int max_iterations = 2000;
        double step_size = 0.5;
        double goal_bias = 0.1;
        double search_radius = 2.0; // Raggio per cercare vicini (Parent Selection & Rewiring)
        
        bool stop_at_goal = false;
        Vertex goal_point = Vertex(0,0);
        double goal_tolerance = 0.5;
    };

    /**
     * @brief Builds an RRT* (Asymptotically Optimal RRT).
     * Implements Parent Selection and Rewiring.
     */
    std::shared_ptr<Roadmap> buildRRTStar(const Map& map, const RRTStarConfig& config);

}

#endif // RRT_STAR_H