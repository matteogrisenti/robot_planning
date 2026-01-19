#ifndef ROADMAP_FACTORY_H
#define ROADMAP_FACTORY_H

#include <memory>
#include <string>
#include "libraries/roadmap.h"
#include "map_library/map_data_structures.h" 

// This function now lives here, breaking the cycle
std::shared_ptr<Roadmap> generateRoadmap(const std::string& type, const Map& map);

#endif // ROADMAP_FACTORY_H