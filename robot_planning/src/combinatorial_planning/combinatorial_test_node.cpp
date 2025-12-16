#include <ros/ros.h>
#include <memory>

#include "map/map_builder.h"
#include "combinatorial_planning/exact_cell_decomposition.h"
#include "combinatorial_planning/approximate_cell_decomposition.h"
#include "combinatorial_planning/maximum_clearance_roadmap.h"
#include "combinatorial_planning/shortest_path_roadmap.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "combinatorial_test_node");
    ros::NodeHandle nh;
    
    ROS_INFO("=== Combinatorial Roadmap Test Node Started ===");
    
    try {
        // Build map once
        ROS_INFO("Building map...");
        map_builder::MapBuilder builder(nh, 100.0);
        Map map = builder.buildMap();
        
        std::string base_path = "src/robot_planning/robot_planning/src/combinatorial_planning/test/";
        
        map.plot(false, true, base_path + "map.png");
        
        ROS_INFO("=== Testing Different Algorithms ===");
        
        // Test 1: Exact Cell Decomposition
        {
            ROS_INFO("\n--- Test 1: Exact Cell Decomposition ---");
            std::shared_ptr<Roadmap> ECD_roadmap; 
            ECD_roadmap = ExactDecomposition::exactCellDecomposition(map);

            if (ECD_roadmap) {
                ROS_INFO("[RoadmapTest] Visualizing roadmap...");
                ECD_roadmap->plot(false, true, base_path + "ECD_roadmap_approx.png");
            } else {
                ROS_WARN("[RoadmapTest] ECD Roadmap generation failed.");
            }
        }

        // Test 2: Approximate Cell Decomposition
        {
            ROS_INFO("\n--- Test 2: Approximate Cell Decomposition ---");
            std::shared_ptr<Roadmap> ACD_roadmap; 
            ACD_roadmap = ApproximateDecomposition::approximateCellDecomposition(map, 5);

            if (ACD_roadmap) {
                ROS_INFO("[RoadmapTest] Visualizing roadmap...");
                ACD_roadmap->plot(false, true, base_path + "ACD_roadmap_approx.png");
            } else {
                ROS_WARN("[RoadmapTest] ACD Roadmap generation failed.");
            }
        }
        
        // Test 3: Maximum Clearance Roadmap
        {
            ROS_INFO("\n--- Test 3: Maximum Clearance Roadmap ---");
            std::shared_ptr<Roadmap> MCR_roadmap; 
            MCR_roadmap = MaxClearanceRoadmap::maximumClearanceRoadmap(map);

            if (MCR_roadmap) {
                ROS_INFO("[RoadmapTest] Visualizing roadmap...");
                MCR_roadmap->plot(false, true, base_path + "MCR_roadmap_approx.png");
            } else {
                ROS_WARN("[RoadmapTest] MCR Roadmap generation failed.");
            }
        }

        // Test 4: Shortest Path Roadmap (se presente)
        {
            ROS_INFO("\n--- Test 4: Shortest Path Roadmap ---");
            // Nota: Assicurati che shortest_path_roadmap.cpp sia compilato nel CMakeLists.txt
            std::shared_ptr<Roadmap> SPR_roadmap;
            SPR_roadmap = generateShortestPathRoadmap(map, 0.2); 

            if (SPR_roadmap) {
                ROS_INFO("[RoadmapTest] Visualizing roadmap...");
                SPR_roadmap->plot(false, true, base_path + "SPR_roadmap_approx.png");
            } else {
                ROS_WARN("[RoadmapTest] SPR Roadmap generation failed.");
            }
        }
                
        ROS_INFO("\n=== All Tests Complete ===");
        ROS_INFO("Check generated PNG files in %s", base_path.c_str());
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in combinatorial_test_node: %s", e.what());
        return 1;
    }
    
    return 0;
}