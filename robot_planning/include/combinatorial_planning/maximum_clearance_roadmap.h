#ifndef MAXIMUM_CLEARANCE_ROADMAP_H
#define MAXIMUM_CLEARANCE_ROADMAP_H

#include <vector>
#include <memory>
#include <map/map_data_structures.h>
#include <roadmap/roadmap_data_structures.h>
#include "planning_utils.h"

namespace MaxClearanceRoadmap {

    // Main Function: Creates paths that maximize distance from obstacles (Voronoi-based)
    std::shared_ptr<Roadmap> maximumClearanceRoadmap(
        const Map& map,
        double gridResolution = 0.5
    );

    // --- Helper Functions ---


}

#endif // MAXIMUM_CLEARANCE_ROADMAP_H