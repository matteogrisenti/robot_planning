#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include "map/map_builder.h"
#include "roadmap/roadmap_data_structures.h"
#include "roadmap/roadmap_visualization.h" // <--- 1. INCLUDE NECESSARIO
#include "combinatorial_planning/exact_cell_decomposition.h"
#include "combinatorial_planning/approximate_cell_decomposition.h"
#include "combinatorial_planning/maximum_clearance_roadmap.h"
#include "sample_based_planning/prm.h"
#include "sample_based_planning/rrt.h"
#include "graph_search/astar.h" 

using namespace std;

shared_ptr<Roadmap> generateRoadmap(const string& type, const Map& map) {
    if (type == "prm") {
        sample_planning::PRMConfig config;
        config.num_samples = 1500; config.k_neighbors = 15;
        return sample_planning::buildPRM(map, config);
    } else if (type == "ecd") return ExactDecomposition::exactCellDecomposition(map);
    else if (type == "acd") return ApproximateDecomposition::approximateCellDecomposition(map, 4);
    else if (type == "mcr") return MaxClearanceRoadmap::maximumClearanceRoadmap(map);
    else if (type == "rrt") {
        sample_planning::RRTConfig config; config.max_iterations = 2000; config.step_size = 1.0;
        return sample_planning::buildRRT(map, config);
    }
    return nullptr;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "astar_planning_test_node");
    ros::NodeHandle nh("~");
    string planner_type;
    nh.param<string>("planner_type", planner_type, "prm");

    try {
        map_builder::MapBuilder builder(nh, 100.0);
        ROS_INFO("Waiting for Map...");
        Map map = builder.buildMap();

        shared_ptr<Roadmap> roadmap = generateRoadmap(planner_type, map);
        if (!roadmap || roadmap->getNumVertices() == 0) return 1;

        // Conversione esplicita Point -> Vertex
        Point s = map.start.get_position();
        Vertex startPose(s.x, s.y);

        Vertex gatePose(0,0);
        if (!map.gates.get_gates().empty()) {
            Point g = map.gates.get_gates()[0].get_position();
            gatePose = Vertex(g.x, g.y);
        }

        vector<int> missionSequence = GraphSearch::TaskPlanner::planMissionSequence(*roadmap, startPose, map.victims.get_victims(), gatePose);
        
        ROS_INFO("Mission Sequence Calculated. Executing A*...");
        
        // --- 2. ACCUMULO DEL PERCORSO ---
        vector<int> fullGlobalPath; 
        bool missionSuccess = true;

        for (size_t i = 0; i < missionSequence.size() - 1; ++i) {
            int from = missionSequence[i];
            int to = missionSequence[i+1];
            vector<int> segment = GraphSearch::AStarPlanner::computePath(*roadmap, from, to);
            
            if (segment.empty()) {
                ROS_ERROR("Unreachable: %d -> %d", from, to);
                missionSuccess = false;
                break;
            } else {
                ROS_INFO("Segment %lu OK: %d -> %d (%lu steps)", i, from, to, segment.size());
                // Uniamo il segmento al percorso globale
                fullGlobalPath.insert(fullGlobalPath.end(), segment.begin(), segment.end());
            }
        }

        // --- 3. VISUALIZZAZIONE FINALE ---
        if (missionSuccess) {
            ROS_INFO("MISSION COMPLETE! Total path nodes: %lu", fullGlobalPath.size());
            ROS_INFO("Generating visualization...");
            
            roadmap_viz::RoadmapVisualizer viz;
            
            // Renderizza base (Map + Roadmap + Vittime)
            viz.render(map, *roadmap);
            
            // Sovrapponi il percorso completo (Rosso)
            viz.drawPath(*roadmap, fullGlobalPath);
            
            // Salva l'immagine
            // NOTA: Assicurati che la cartella 'test_results' esista dentro src/robot_planning/
            string filename = "src/robot_planning/robot_planning/src/graph_search/test/mission_" + planner_type + ".png";
            if (viz.saveToFile(filename)) {
                ROS_INFO("Visualization saved to: %s", filename.c_str());
            }
        }

    } catch (const exception& e) { ROS_ERROR("%s", e.what()); return 1; }
    return 0;
}