#include <ros/ros.h>
#include <memory>
#include <string>

// Include Map & Roadmap Infrastructure
#include "map/map_builder.h"
#include "map/map_data_structures.h"
#include "roadmap/roadmap_data_structures.h"

// Include Sample-Based Algorithms
#include "sample_based_planning/prm.h"
#include "sample_based_planning/rrt.h"
#include "sample_based_planning/rrt_star.h"

// TODO implemnt path smoothing 

int main(int argc, char **argv)
{
    // 1. ROS Initialization
    ros::init(argc, argv, "sample_planning_test_node");
    ros::NodeHandle nh;

    ROS_INFO("=== Sample-Based Planning Test Node Started ===");

    try {
        // ---------------------------------------------------------
        // A. BUILD ENVIRONMENT (Map)
        // ---------------------------------------------------------
        ROS_INFO("1. Waiting for Map Data...");
        // Timeout 10s. Assicurati che la simulazione stia pubblicando i topic!
        map_builder::MapBuilder builder(nh, 100.0); 
        Map map = builder.buildMap();
        
        // Define base output directory for tests
        // NOTE: Assicurati che questa cartella esista o creala con mkdir -p
        std::string image_output_dir = "src/robot_planning/robot_planning/src/sample_based_planning/test/map.png";
        ROS_INFO("Map built successfully. Saving base map image...");
        map.plot(false, true, image_output_dir);

        // ---------------------------------------------------------
        // B. TEST 1: PROBABILISTIC ROADMAP (PRM)
        // ---------------------------------------------------------
        {
            ROS_INFO("\n--- Running Algorithm: PRM ---");

            // 1. Configuration
            sample_planning::PRMConfig config;
            config.num_samples = 1000;   // Aumenta se la mappa è grande
            config.k_neighbors = 10;    // Connettività locale
            config.max_connection_dist = -1.0; // Illimitato, o metti valore in metri (es. 5.0)

            // 2. Execution
            std::shared_ptr<Roadmap> prm_roadmap = sample_planning::buildPRM(map, config);

            // 3. Visualization & Output
            if (prm_roadmap && prm_roadmap->getNumVertices() > 0) {
                std::string output_file = "src/robot_planning/robot_planning/src/sample_based_planning/test/prm.png";
                ROS_INFO("PRM generated with %d vertices. Saving to: %s", 
                         prm_roadmap->getNumVertices(), output_file.c_str());
                
                // plot(display=false, save=true, filename)
                prm_roadmap->plot(false, true, output_file);
            } else {
                ROS_WARN("PRM failed to generate a valid roadmap (0 vertices). Check bounds or obstacles.");
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
            
            // Setup Goal (Opzionale: usiamo il primo gate se c'è)
            if (!map.gates.get_gates().empty()) {
                Point g = map.gates.get_gates()[0].get_position();
                config.goal_point = Vertex(g.x, g.y);
                config.stop_at_goal = false; // Mettiamo false per vedere l'albero espandersi ovunque
                config.goal_bias = 0.1;
            }

            auto rrt = sample_planning::buildRRT(map, config);
            if (rrt) {
                // Visualizza
                std::string output_file = "src/robot_planning/robot_planning/src/sample_based_planning/test/rrt.png";
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
            config.search_radius = 2.0; // Raggio di rewiring

            // Opzionale: Setup Goal
            if (!map.gates.get_gates().empty()) {
                Point g = map.gates.get_gates()[0].get_position();
                config.goal_point = Vertex(g.x, g.y);
                config.stop_at_goal = false; // Lascia false per vedere l'ottimizzazione globale
            }

            auto rrt_star = sample_planning::buildRRTStar(map, config);
            if (rrt_star) {
                std::string output_file = "src/robot_planning/robot_planning/src/sample_based_planning/test/rrt_star.png";
                rrt_star->plot(false, true, output_file);
            }
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("CRITICAL ERROR in sample_planning_test_node: %s", e.what());
        return 1;
    }

    ROS_INFO("\n=== Test Complete. Shutting down node. ===");
    return 0;
}