#include <ros/ros.h>
#include <memory>
#include <string>
#include <iostream>


#include "map_library.h"
#include "libraries/roadmap.h"
#include "libraries/combinatorial_planning.h"
#include "libraries/sample_based_planning.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_roadmap_unified");
    ros::NodeHandle nh;

    ROS_INFO("=== Unified Roadmap Test Node Started ===");

    try {
        // ---------------------------------------------------------
        // A. BUILD ENVIRONMENT 
        // ---------------------------------------------------------
        ROS_INFO("1. Waiting for Map Data...");
        map_builder::MapBuilder builder(nh, 1000.0);
        Map map = builder.buildMap();

        std::string combinatorial_base_path = "src/robot_planning/robot_planning/src/libraries/combinatorial_planning/test/";
        std::string sample_base_path = "src/robot_planning/robot_planning/src/libraries/sample_based_planning/test/";

        ROS_INFO("Map built successfully.");
        
        map.plot(false, true, combinatorial_base_path + "map.png");
        map.plot(false, true, sample_base_path + "map.png");

        // =========================================================
        // SECTION 1: COMBINATORIAL PLANNING
        // =========================================================
        ROS_INFO("\n=== SECTION 1: COMBINATORIAL PLANNING ALGORITHMS ===");

        // Test 1.1: Exact Cell Decomposition
        {
            ROS_INFO("--- Test 1.1: Exact Cell Decomposition ---");
            std::shared_ptr<Roadmap> ECD_roadmap = exactCellDecomposition(map);

            if (ECD_roadmap) {
                std::string output_file = combinatorial_base_path + "ECD_roadmap_approx.png";
                ECD_roadmap->plot(false, true, output_file);
                ROS_INFO("Saved: %s", output_file.c_str());
            } else {
                ROS_WARN("ECD Roadmap generation failed.");
            }
        }

        // Test 1.2: Approximate Cell Decomposition
        {
            ROS_INFO("--- Test 1.2: Approximate Cell Decomposition ---");
            std::shared_ptr<Roadmap> ACD_roadmap = approximateCellDecomposition(map, 5);

            if (ACD_roadmap) {
                std::string output_file = combinatorial_base_path + "ACD_roadmap_approx.png";
                ACD_roadmap->plot(false, true, output_file);
                ROS_INFO("Saved: %s", output_file.c_str());
            } else {
                ROS_WARN("ACD Roadmap generation failed.");
            }
        }

        // Test 1.3: Maximum Clearance Roadmap
        {
            ROS_INFO("--- Test 1.3: Maximum Clearance Roadmap ---");
            std::shared_ptr<Roadmap> MCR_roadmap = maximumClearanceRoadmap(map);

            if (MCR_roadmap) {
                std::string output_file = combinatorial_base_path + "MCR_roadmap_approx.png";
                MCR_roadmap->plot(false, true, output_file);
                ROS_INFO("Saved: %s", output_file.c_str());
            } else {
                ROS_WARN("MCR Roadmap generation failed.");
            }
        }

        // Test 1.4: Shortest Path Roadmap
        {
            ROS_INFO("--- Test 1.4: Shortest Path Roadmap ---");
            double padding = 0.1; 
            std::shared_ptr<Roadmap> SPR_roadmap = shortestPathRoadmap(map, padding);

            if (SPR_roadmap) {
                std::string output_file = combinatorial_base_path + "SPR_roadmap_approx.png";
                SPR_roadmap->plot(false, true, output_file);
                ROS_INFO("Saved: %s", output_file.c_str());
            } else {
                ROS_WARN("SPR Roadmap generation failed.");
            }
        }

        // =========================================================
        // SECTION 2: SAMPLE-BASED PLANNING
        // =========================================================
        ROS_INFO("\n=== SECTION 2: SAMPLE-BASED PLANNING ALGORITHMS ===");

        // Test 2.1: Probabilistic Roadmap (PRM)
        {
            ROS_INFO("--- Test 2.1: PRM ---");
            sample_planning::PRMConfig config;
            config.num_samples = 1000;
            config.k_neighbors = 10;
            config.max_connection_dist = -1.0;

            std::shared_ptr<Roadmap> prm_roadmap = sample_planning::buildPRM(map, config);

            if (prm_roadmap && prm_roadmap->getNumVertices() > 0) {
                std::string output_file = sample_base_path + "prm.png";
                prm_roadmap->plot(false, true, output_file);
                ROS_INFO("Saved: %s (Vertices: %d)", output_file.c_str(), prm_roadmap->getNumVertices());
            } else {
                ROS_WARN("PRM generation failed or empty.");
            }
        }

        // Test 2.2: RRT
        {
            ROS_INFO("--- Test 2.2: RRT ---");
            sample_planning::RRTConfig config;
            config.max_iterations = 2000;
            config.step_size = 1.0;
            
            if (!map.gates.get_gates().empty()) {
                Point g = map.gates.get_gates()[0].get_position();
                config.goal_point = Vertex(g.x, g.y);
                config.stop_at_goal = false; 
                config.goal_bias = 0.1;
            }

            auto rrt = sample_planning::buildRRT(map, config);
            if (rrt) {
                std::string output_file = sample_base_path + "rrt.png";
                rrt->plot(false, true, output_file);
                ROS_INFO("Saved: %s", output_file.c_str());
            }
        }

        // Test 2.3: RRT*
        {
            ROS_INFO("--- Test 2.3: RRT* ---");
            sample_planning::RRTStarConfig config;
            config.max_iterations = 2000;
            config.step_size = 0.4; 
            config.search_radius = 1.2; 

            if (!map.gates.get_gates().empty()) {
                Point g = map.gates.get_gates()[0].get_position();
                config.goal_point = Vertex(g.x, g.y);
                config.stop_at_goal = false;
            }

            auto rrt_star = sample_planning::buildRRTStar(map, config);
            if (rrt_star) {
                std::string output_file = sample_base_path + "rrt_star.png";
                rrt_star->plot(false, true, output_file);
                ROS_INFO("Saved: %s", output_file.c_str());
            }
        }

    } catch (const std::exception& e) {
        ROS_ERROR("CRITICAL ERROR in test_roadmap_unified: %s", e.what());
        return 1;
    }

    ROS_INFO("\n=== All Unified Tests Complete. Shutting down. ===");
    return 0;
}