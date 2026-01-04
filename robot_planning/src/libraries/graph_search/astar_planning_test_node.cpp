#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <iomanip>
#include <iostream>

#include "map_library/map_builder.h"
#include "roadmap.h"
#include "combinatorial_planning.h"
#include "sample_based_planning.h"
#include "astar.h" 
#include "roadmap_factory.h"

using namespace std;
using namespace std::chrono;

// Struttura per salvare i dati del benchmark
struct BenchmarkResult {
    string name;
    double roadmap_time_ms;
    double search_time_ms;
    bool success;
    size_t path_length;
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "astar_planning_benchmark_node");
    ros::NodeHandle nh("~");

    // Lista di tutti i planner da testare
    vector<string> planner_types = {
        "prm", "rrt", "rrt_star", "ecd", "acd", "mcr", "spr"
    };
    
    vector<BenchmarkResult> report;

    try {
        // 1. Costruisci la Mappa
        map_builder::MapBuilder builder(nh, 1000.0);
        ROS_INFO("Waiting for Map Data...");
        Map map = builder.buildMap();
        ROS_INFO("Map built successfully. Starting Benchmark Suite...");

        // Prepara coordinate Start e Gate
        Point s = map.start.get_position();
        Vertex startPose(s.x, s.y);
        Vertex gatePose(0,0);
        if (!map.gates.get_gates().empty()) {
            Point g = map.gates.get_gates()[0].get_position();
            gatePose = Vertex(g.x, g.y);
        }

        // ================= LOOP SU TUTTI I PLANNER =================
        for (const string& planner_type : planner_types) {
            ROS_INFO("\n------------------------------------------------");
            ROS_INFO(">>> TESTING PLANNER: %s", planner_type.c_str());
            
            BenchmarkResult res;
            res.name = planner_type;
            res.success = false;
            res.path_length = 0;

            // --- 1. Misura Tempo Roadmap ---
            auto t1 = high_resolution_clock::now();
            shared_ptr<Roadmap> roadmap = generateRoadmap(planner_type, map);
            auto t2 = high_resolution_clock::now();
            res.roadmap_time_ms = duration<double, milli>(t2 - t1).count();

            if (!roadmap || roadmap->getNumVertices() == 0) {
                ROS_ERROR("Roadmap generation FAILED for %s", planner_type.c_str());
                report.push_back(res);
                continue;
            }
            ROS_INFO("Roadmap Generated: %d vertices in %.2f ms", roadmap->getNumVertices(), res.roadmap_time_ms);

            // --- 2. Misura Tempo Ricerca (Task + A*) ---
            auto t3 = high_resolution_clock::now();
            
            // Task Planning
            // FIX: Aggiunti time_limit (120.0s) e robot_velocity (0.5 m/s) di default per il benchmark
            vector<int> missionSequence = GraphSearch::TaskPlanner::planMissionSequence(
                *roadmap, startPose, map.victims.get_victims(), gatePose, 120.0, 0.5
            );
            
            // A* Pathfinding
            vector<int> fullGlobalPath;
            bool pathSuccess = true;
            if (missionSequence.empty()) pathSuccess = false;

            for (size_t i = 0; i < missionSequence.size() - 1; ++i) {
                vector<int> segment = GraphSearch::AStarPlanner::computePath(*roadmap, missionSequence[i], missionSequence[i+1]);
                if (segment.empty()) {
                    ROS_WARN("Segment %lu unreachable (%d -> %d)", i, missionSequence[i], missionSequence[i+1]);
                    pathSuccess = false; 
                    break;
                }
                fullGlobalPath.insert(fullGlobalPath.end(), segment.begin(), segment.end());
            }
            auto t4 = high_resolution_clock::now();
            res.search_time_ms = duration<double, milli>(t4 - t3).count();
            
            // --- 3. SALVATAGGIO IMMAGINE ---
            if (pathSuccess && !fullGlobalPath.empty()) {
                res.success = true;
                res.path_length = fullGlobalPath.size();
                ROS_INFO("Search COMPLETE. Path len: %lu. Time: %.2f ms", res.path_length, res.search_time_ms);

                // Visualizzazione
                roadmap_viz::RoadmapVisualizer viz;
                viz.render(map, *roadmap);
                viz.drawPath(*roadmap, fullGlobalPath);
                
                // Crea cartella se non esiste
                string output_dir = "src/robot_planning/src/libraries/graph_search/test/";
                // system(("mkdir -p " + output_dir).c_str());
                
                // Nome file univoco per ogni planner
                string filename = output_dir + "mission_" + planner_type + ".png";
                
                if (viz.saveToFile(filename)) {
                    ROS_INFO("SUCCESS: Saved image to %s", filename.c_str());
                } else {
                    ROS_ERROR("FAILED to save image to %s", filename.c_str());
                }
            } else {
                ROS_WARN("Search FAILED for %s", planner_type.c_str());
            }

            report.push_back(res);
        }

        // ================= REPORT FINALE =================
        cout << "\n\n";
        cout << "========================================================================\n";
        cout << "                       PLANNING BENCHMARK REPORT                        \n";
        cout << "========================================================================\n";
        cout << left << setw(12) << "PLANNER" 
             << setw(15) << "ROADMAP (ms)" 
             << setw(15) << "SEARCH (ms)" 
             << setw(15) << "TOTAL (ms)" 
             << setw(10) << "STATUS" 
             << "PATH NODES\n";
        cout << "------------------------------------------------------------------------\n";
        
        for (const auto& r : report) {
            double total = r.roadmap_time_ms + r.search_time_ms;
            cout << left << setw(12) << r.name 
                 << fixed << setprecision(2) << setw(15) << r.roadmap_time_ms 
                 << setw(15) << r.search_time_ms 
                 << setw(15) << total
                 << setw(10) << (r.success ? "OK" : "FAIL") 
                 << r.path_length << "\n";
        }
        cout << "========================================================================\n";

    } catch (const exception& e) {
        ROS_ERROR("CRITICAL ERROR: %s", e.what());
        return 1;
    }

    return 0;
}