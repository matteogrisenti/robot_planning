#include <ros/ros.h>
#include <memory>

#include "map/map_builder.h"
#include "combinatorial_planning/exact_cell_decomposition.h"
#include "combinatorial_planning/approximate_cell_decomposition.h"
#include "combinatorial_planning/maximum_clearance_roadmap.h"

/**
 * @brief Test node for experimenting with different roadmap algorithms
 * 
 * This node allows you to test and compare different roadmap generation
 * algorithms on the same map data.
 */
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
        
        // Display the map
        std::string output_map_file = "src/robot_planning/src/combinatorial_planning/test/map.png";
        map.plot(false, true, output_map_file);
        
        ROS_INFO("=== Testing Different Algorithms ===");
        
        // Test 1: Maximum Clearance Roadmap
        {
            ROS_INFO("\n--- Test 1: Exact Cell Decomposition ---");
            std::shared_ptr<Roadmap> ECD_roadmap; 
            ECD_roadmap = ExactDecomposition::exactCellDecomposition(map);

            if (!ECD_roadmap) {
                ROS_WARN("[RoadmapTest] Cannot visualize null roadmap");
                return 1;
            }
            
            ROS_INFO("[RoadmapTest] Visualizing roadmap...");
            
            // Display or save
            std::string output_file = "src/robot_planning/src/combinatorial_planning/test/ECD_roadmap_approx.png";
            ECD_roadmap->plot(false, true, output_file);
        }
        
        // Test 2: Exact Cell Decomposition
        {
            ROS_INFO("\n--- Test 2: Approximate Cell Decomposition ---");
            std::shared_ptr<Roadmap> ACD_roadmap; 
            ACD_roadmap = ApproximateDecomposition::approximateCellDecomposition(map, 5);

            if (!ACD_roadmap) {
                ROS_WARN("[RoadmapTest] Cannot visualize null roadmap");
                return 1;
            }
            
            ROS_INFO("[RoadmapTest] Visualizing roadmap...");
            
            // Display or save
            std::string output_file = "src/robot_planning/src/combinatorial_planning/test/ACD_roadmap_approx.png";
            ACD_roadmap->plot(false, true, output_file);
        }
        
        // Test 3: Approximate Cell Decomposition (different grid sizes)
        {
            ROS_INFO("\n--- Test 3: Approximate Cell Decomposition");
            std::shared_ptr<Roadmap> MCR_roadmap; 
            MCR_roadmap = MaxClearanceRoadmap::maximumClearanceRoadmap(map);

            if (!MCR_roadmap) {
                ROS_WARN("[RoadmapTest] Cannot visualize null roadmap");
                return 1;
            }
            
            ROS_INFO("[RoadmapTest] Visualizing roadmap...");
            
            // Display or save
            std::string output_file = "src/robot_planning/src/combinatorial_planning/test/MCR_roadmap_approx.png";
            MCR_roadmap->plot(false, true, output_file);
        }
        
        ROS_INFO("\n=== All Tests Complete ===");
        ROS_INFO("Check generated PNG files for visualization results");
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in roadmap_test_node: %s", e.what());
        return 1;
    }
    
    return 0;
}