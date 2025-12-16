#include <ros/ros.h>
#include <memory>

#include "map/map_builder.h"
#include "combinatorial_planning/exact_cell_decomposition.h"
#include "combinatorial_planning/approximate_cell_decomposition.h"
#include "combinatorial_planning/maximum_clearance_roadmap.h"
#include "combinatorial_planning/shortest_path_roadmap.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roadmap_test_node");
    ros::NodeHandle nh;
    
    ROS_INFO("=== Roadmap Test Node Started ===");
    
    try {
        // Build map once
        ROS_INFO("Building map...");
        map_builder::MapBuilder builder(nh, 100.0);
        Map map = builder.buildMap();
        
        // CORREZIONE PATH: src/robot_planning/robot_planning/src/...
        std::string base_path = "src/robot_planning/robot_planning/src/combinatorial_planning/test/";
        
        map.plot(false, true, base_path + "map.png");
        
        ROS_INFO("=== Testing Different Algorithms ===");
        
        // Test 1: Exact Cell Decomposition
        {
            ROS_INFO("\n--- Test 1: Exact Cell Decomposition ---");
            std::shared_ptr<Roadmap> ECD_roadmap; 
            ECD_roadmap = exactCellDecomposition(map);

            if (ECD_roadmap) {
                ROS_INFO("[RoadmapTest] Visualizing roadmap...");
                ECD_roadmap->plot(false, true, base_path + "ECD_roadmap_approx.png");
            }
        }
<<<<<<< HEAD:robot_planning/src/combinatorial_planning/roadmap_test_node.cpp
        
=======

>>>>>>> e033b818ae5240edf046b6df58f6f8aed5d7eda7:robot_planning/src/combinatorial_planning/combinatorial_test_node.cpp
        // Test 2: Approximate Cell Decomposition
        {
            ROS_INFO("\n--- Test 2: Approximate Cell Decomposition ---");
            std::shared_ptr<Roadmap> ACD_roadmap; 
            ACD_roadmap = approximateCellDecomposition(map, 5);

            if (ACD_roadmap) {
                ROS_INFO("[RoadmapTest] Visualizing roadmap...");
                ACD_roadmap->plot(false, true, base_path + "ACD_roadmap_approx.png");
            }
        }
        
<<<<<<< HEAD:robot_planning/src/combinatorial_planning/roadmap_test_node.cpp
        // Test 3: Maximum Clearance
        {
            ROS_INFO("\n--- Test 3: Maximum Clearance Roadmap ---");
            std::shared_ptr<Roadmap> MCR_roadmap; 
            MCR_roadmap = MaxClearanceRoadmap::maximumClearanceRoadmap(map);

            if (MCR_roadmap) {
                ROS_INFO("[RoadmapTest] Visualizing roadmap...");
                MCR_roadmap->plot(false, true, base_path + "MCR_roadmap_approx.png");
=======
        // Test 3: Maximum Clearance Roadmap (Voronoi)
        {
            ROS_INFO("\n--- Test 3: Maximum Clearance Roadmap ---");
            
            std::shared_ptr<Roadmap> MCR_roadmap;
            MCR_roadmap = generateMaxClearanceRoadmap(map);
            
            if (!MCR_roadmap) {
                ROS_WARN("[RoadmapTest] Cannot visualize null roadmap");
                return 1;
>>>>>>> e033b818ae5240edf046b6df58f6f8aed5d7eda7:robot_planning/src/combinatorial_planning/combinatorial_test_node.cpp
            }
        }

        // Test 4: Shortest Path Roadmap
        {
            ROS_INFO("\n--- Test 4: Shortest Path Roadmap ---");
            
            std::shared_ptr<Roadmap> SPR_roadmap;
            SPR_roadmap = generateShortestPathRoadmap(map, 0.2); // Example padding of 0.2 units

            if (!SPR_roadmap) {
                ROS_WARN("[RoadmapTest] Cannot visualize null roadmap");
                return 1;
            }

            ROS_INFO("[RoadmapTest] Visualizing roadmap...");

            // Display or save
            std::string output_file = "src/robot_planning/src/combinatorial_planning/test/SPR_roadmap_approx.png";
            SPR_roadmap->plot(false, true, output_file);
        }
                
        ROS_INFO("\n=== All Tests Complete ===");
        ROS_INFO("Check generated PNG files in %s", base_path.c_str());
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in roadmap_test_node: %s", e.what());
        return 1;
    }
    
    return 0;
}