#ifndef RRT_H
#define RRT_H

#include <memory>
#include <map_library/map_data_structures.h>
#include <libraries/roadmap.h>

namespace sample_planning {

    struct RRTConfig {
        int max_iterations = 2000;  
        double step_size = 0.5;     
        double goal_bias = 0.1;     
        
        bool stop_at_goal = false;  
        Vertex goal_point = Vertex(0,0); 
        double goal_tolerance = 1.0;     
    };

    /**
     * @brief Builds an RRT (Rapidly-exploring Random Tree) following the standard algorithm.
     *
     */
    std::shared_ptr<Roadmap> buildRRT(const Map& map, const RRTConfig& config);

}

#endif // RRT_H