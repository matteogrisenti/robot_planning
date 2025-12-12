#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>

// Map Builder
#include "map/map_builder.h"

// Algoritmi Sample Based
#include "sample_based_planning/prm.h"

/**
 * @brief Nodo di test per algoritmi di Sample-Based Planning (PRM, RRT, RRT*)
 * * Carica la mappa e genera roadmaps basate su campionamento e curve di Dubins.
 * Salva i risultati come immagini PNG.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sampling_planning_test_node");
    ros::NodeHandle nh;
    
    ROS_INFO("=== Sampling Planning Test Node Started ===");
    
    try {
        // --------------------------------------------------------
        // 1. Costruzione Mappa (Broadcasting dei dati dell'ambiente)
        // --------------------------------------------------------
        ROS_INFO("Building map...");
        // Timeout di 10s per essere sicuri di ricevere ostacoli e bordi
        map_builder::MapBuilder builder(nh, 100.0); 
        Map map = builder.buildMap();
        
        // Plot della mappa "pulita" per riferimento
        // Nota: Assicurati che la cartella di output esista
        std::string output_base_path = "src/robot_planning/robot_planning/src/sample_based_planning/test/";
        std::string map_file = output_base_path + "map.png";
        
        ROS_INFO("Saving base map to: %s", map_file.c_str());
        map.plot(false, true, map_file);
        
        ROS_INFO("=== Testing Algorithms ===");
        
        // --------------------------------------------------------
        // TEST 1: Probabilistic Roadmap (PRM)
        // --------------------------------------------------------
        {
            ROS_INFO("\n--- Test 1: Probabilistic Roadmap (PRM) ---");
            
            // --- Parametri di Configurazione ---
            int num_samples = 500;       // Numero di nodi da campionare
            double connection_rad = 5.0; // Raggio max per tentare connessione (Euclideo)
            double rho = 1.0;            // Raggio minimo di curvatura (Dubins) -> Kmax = 1.0
            double robot_rad = 0.25;     // Raggio robot + margine sicurezza
            
            ROS_INFO("Parameters: Samples=%d, ConnRadius=%.2f, Rho=%.2f, RobotRadius=%.2f", 
                     num_samples, connection_rad, rho, robot_rad);

            // Istanziazione Planner
            SampleBasedPlanning::PRMPlanner prm(num_samples, connection_rad, rho, robot_rad);

            // Esecuzione e Timing
            auto start = ros::Time::now();
            std::shared_ptr<Roadmap> prm_roadmap = prm.buildRoadmap(map);
            auto end = ros::Time::now();
            
            if (prm_roadmap) {
                double duration = (end - start).toSec();
                ROS_INFO("[PRM] Roadmap generated in %.4f seconds", duration);
                ROS_INFO("[PRM] Total Vertices: %d", prm_roadmap->getNumVertices());
                
                // Visualizzazione
                std::string output_file = output_base_path + "PRM_roadmap_approx.png";
                ROS_INFO("[PRM] Saving visualization to: %s", output_file.c_str());
                prm_roadmap->plot(false, true, output_file);
            } else {
                ROS_WARN("[PRM] Failed to generate roadmap (null pointer returned).");
            }
        }
        
        // --------------------------------------------------------
        // TEST 2: RRT (Placeholder per implementazione futura)
        // --------------------------------------------------------
        /*
        {
            ROS_INFO("\n--- Test 2: Rapidly-exploring Random Tree (RRT) ---");
            // TODO: Implementare RRTPlanner e decommentare
        }
        */

        ROS_INFO("\n=== All Tests Complete ===");
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in sampling_planning_test_node: %s", e.what());
        return 1;
    }
    
    return 0;
}