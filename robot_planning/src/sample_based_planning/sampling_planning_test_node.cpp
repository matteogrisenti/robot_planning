#include <ros/ros.h>
#include <memory>
#include <string>
#include <chrono> // <--- NECESSARIO PER I TIMER
#include <iomanip> // Per formattazione output

// Include Map & Roadmap Infrastructure
#include "map_library.h"
#include "roadmap/roadmap_data_structures.h"

// Include Sample-Based Algorithms
#include "sample_based_planning/prm.h"
#include "sample_based_planning/rrt.h"
#include "sample_based_planning/rrt_star.h"

int main(int argc, char **argv)
{
    // 1. ROS Initialization
    ros::init(argc, argv, "sample_planning_test_node");
    ros::NodeHandle nh;

    ROS_INFO("=== Sample-Based Planning Test Node Started ===");

    // Variabili per salvare i tempi
    double time_prm = 0.0;
    double time_rrt = 0.0;
    double time_rrt_star = 0.0;

    try {
        // ---------------------------------------------------------
        // A. BUILD ENVIRONMENT (Map)
        // ---------------------------------------------------------
        ROS_INFO("1. Waiting for Map Data...");
        map_builder::MapBuilder builder(nh, 1000.0); 
        Map map = builder.buildMap();
        
        // Define base output directory for tests
        std::string image_output_dir = "src/robot_planning/src/sample_based_planning/test/map.png";
        ROS_INFO("Map built successfully. Saving base map image...");
        map.plot(false, true, image_output_dir);

        // ---------------------------------------------------------
        // B. TEST 1: PROBABILISTIC ROADMAP (PRM)
        // ---------------------------------------------------------
        {
            ROS_INFO("\n--- Running Algorithm: PRM ---");

            sample_planning::PRMConfig config;
            config.num_samples = 1000;
            config.k_neighbors = 10;
            config.max_connection_dist = -1.0;

            // TIMER START
            auto start = std::chrono::high_resolution_clock::now();
            
            std::shared_ptr<Roadmap> prm_roadmap = sample_planning::buildPRM(map, config);
            
            // TIMER END
            auto end = std::chrono::high_resolution_clock::now();
            time_prm = std::chrono::duration<double>(end - start).count();

            if (prm_roadmap && prm_roadmap->getNumVertices() > 0) {
                std::string output_file = "src/robot_planning/src/sample_based_planning/test/prm.png";
                ROS_INFO("PRM generated with %d vertices in %.4f s. Saving...", 
                         prm_roadmap->getNumVertices(), time_prm);
                prm_roadmap->plot(false, true, output_file);
            } else {
                ROS_WARN("PRM failed.");
            }
        }

        // ---------------------------------------------------------
        // TEST RRT
        // ---------------------------------------------------------
        {
            ROS_INFO("\n--- Running Algorithm: RRT ---");
            sample_planning::RRTConfig config;
            config.max_iterations = 2000; 
            config.step_size = 1.0;       
            
            if (!map.gates.get_gates().empty()) {
                Point g = map.gates.get_gates()[0].get_position();
                config.goal_point = Vertex(g.x, g.y);
                config.stop_at_goal = false;
                config.goal_bias = 0.1;
            }

            // TIMER START
            auto start = std::chrono::high_resolution_clock::now();

            auto rrt = sample_planning::buildRRT(map, config);

            // TIMER END
            auto end = std::chrono::high_resolution_clock::now();
            time_rrt = std::chrono::duration<double>(end - start).count();

            if (rrt) {
                std::string output_file = "src/robot_planning/src/sample_based_planning/test/rrt.png";
                ROS_INFO("RRT generated in %.4f s. Saving...", time_rrt);
                rrt->plot(false, true, output_file);
            }
        }

        // ---------------------------------------------------------
        // TEST RRT*
        // ---------------------------------------------------------
        {
            ROS_INFO("\n--- Running Algorithm: RRT* ---");
            sample_planning::RRTStarConfig config;
            config.max_iterations = 2000;
            config.step_size = 1.0;
            config.search_radius = 2.0;

            if (!map.gates.get_gates().empty()) {
                Point g = map.gates.get_gates()[0].get_position();
                config.goal_point = Vertex(g.x, g.y);
                config.stop_at_goal = false;
            }

            // TIMER START
            auto start = std::chrono::high_resolution_clock::now();

            auto rrt_star = sample_planning::buildRRTStar(map, config);

            // TIMER END
            auto end = std::chrono::high_resolution_clock::now();
            time_rrt_star = std::chrono::duration<double>(end - start).count();

            if (rrt_star) {
                std::string output_file = "src/robot_planning/src/sample_based_planning/test/rrt_star.png";
                ROS_INFO("RRT* generated in %.4f s. Saving...", time_rrt_star);
                rrt_star->plot(false, true, output_file);
            }
        }

        // ---------------------------------------------------------
        // FINAL PERFORMANCE REPORT
        // ---------------------------------------------------------
        std::cout << "\n=========================================\n";
        std::cout << "      ROADMAP GENERATION PERFORMANCE      \n";
        std::cout << "=========================================\n";
        std::cout << "Algorithm  | Time (s)  \n";
        std::cout << "-----------|-----------\n";
        std::cout << std::left << std::setw(11) << "PRM" << "| " << std::fixed << std::setprecision(4) << time_prm << "\n";
        std::cout << std::left << std::setw(11) << "RRT" << "| " << std::fixed << std::setprecision(4) << time_rrt << "\n";
        std::cout << std::left << std::setw(11) << "RRT*" << "| " << std::fixed << std::setprecision(4) << time_rrt_star << "\n";
        std::cout << "=========================================\n";
        
    } catch (const std::exception& e) {
        ROS_ERROR("CRITICAL ERROR in sample_planning_test_node: %s", e.what());
        return 1;
    }

    ROS_INFO("\n=== Test Complete. Shutting down node. ===");
    return 0;
}