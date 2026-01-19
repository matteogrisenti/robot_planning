#include <ros/ros.h>
#include <string>
#include <fstream>
#include <iostream>

#include "mission_manager.h"

const std::string OUTPUT_DIR = "src/robot_planning/src/test/";
const std::string METRICS_FILENAME = "benchmark_results_2.txt";

void appendMetricsToFile(const std::string& filepath, const RunMetrics& m) {
    std::ofstream file;
    file.open(filepath, std::ios_base::app); 
    
    if (file.is_open()) {
        file.seekp(0, std::ios::end);
        if (file.tellp() == 0) {
            file << "Planner\tLimit(s)\tScore\tExec_T(s)\tPlan_T(s)\tVictims\tRetries\tSuccess\n";
        }
        
        file << m.planner << "\t" 
             << m.time_limit_set << "\t"
             << m.total_score << "\t"       
             << m.t_execution << "\t"       
             << m.t_total_planning << "\t"
             << m.victims_collected << "\t"
             << m.dubins_retries << "\t"
             << (m.success ? "YES" : "NO") << "\n";
             
        ROS_INFO("Metrics saved to: %s", filepath.c_str());
        file.close();
    } else {
        ROS_ERROR("Unable to open metrics file: %s", filepath.c_str());
    }
}

int main(int argc, char **argv) {
    ROS_WARN("=== BENCHMARK STARTED ===");
    ros::init(argc, argv, "individual_benchmark_node");
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(2); 
    spinner.start();

    // --- PARAMETER CONFIG ---
    std::string planner_type;
    nh.param<std::string>("planner_type", planner_type, "acd");
    
    double time_limit;
    nh.param<double>("time_limit", time_limit, 120.0); 
    
    ROS_INFO(">>> SELECTED PLANNER: %s", planner_type.c_str());
    ROS_INFO(">>> TIME LIMIT: %.1f s", time_limit);

    // --- INSTANTIATE MANAGER ---
    MissionManager missionManager(nh);

    // --- RUN MISSION ---
    // The manager handles map generation, planning, and execution
    RunMetrics results = missionManager.run(planner_type, time_limit, OUTPUT_DIR);

    // --- LOGGING ---
    ROS_INFO("\n=== MISSION REPORT ===");
    ROS_INFO("Planner: %s | Limit: %.1f s", results.planner.c_str(), results.time_limit_set);
    ROS_INFO("Total Score: %.1f", results.total_score);
    ROS_INFO("Time Exec: %.3fs", results.t_execution);
    ROS_INFO("Victims: %d | Success: %s", results.victims_collected, results.success ? "YES" : "NO");

    appendMetricsToFile(OUTPUT_DIR + METRICS_FILENAME, results);

    return 0;
}