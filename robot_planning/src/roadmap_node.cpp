// This file read the map border, obstacle and victims from the map_pck:
//  -   /obstacle
//  -   /map_borders
//  -   /victims
// Then it i

#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>

#include "roadmap/data_structures.h"


/* ===================================== Obstacle Processing ======================================== */
// This function read the message containing the obstacles data coming from the map package. 
// Extract this data and store the inside the Obstacles attributes of the give Roadmap element.
void processObstacles(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg, Roadmap& roadmap) {
    // obstacles_msgs/ObstacleArrayMsg
    // header: ...
    // obstacles:
    //  -
    //      header: ...
    //      polygon:
    //          points: 
    //              -
    //               x: ...
    //               y: ...
    //               z: ...
    //              - 
    //               ...
    //          radius: ...

    ROS_INFO("[Obstacles] Received %zu obstacles:", msg->obstacles.size());
    
    for (const auto& obstacle_msg : msg->obstacles) {
        // Convert ROS message points to your Point type
        std::vector<Point> points;
        points.reserve(obstacle_msg.polygon.points.size());
        
        for (const auto& point_msg : obstacle_msg.polygon.points) {
            points.push_back({point_msg.x, point_msg.y, point_msg.z});
        }
        
        // Get radius from message
        float radius = obstacle_msg.radius;
        
        // Add obstacle to roadmap
        roadmap.obstacles.add_obstacle(&points, radius);
    }

    ROS_INFO_STREAM("[Obstacles] Roadmap obstacles: " << roadmap.obstacles.to_string());
}

/* ================================== Map Borders Processing ======================================== */
// This function read the message containing the map borders data coming from the map package. 
// Extract this data and store the inside the MapBoardes attributes of the give Roadmap element.
void processMapBorders(const geometry_msgs::Polygon::ConstPtr& msg, Roadmap& roadmap){ 
    // geometry_msgs/Polygon
    // points:
    //      -
    //       x: ...
    //       y: ...
    //       z: ...

    ROS_INFO("[MapBorders] Received %zu vertices:", msg->points.size());

    for (size_t i = 0; i < msg->points.size(); ++i){
        const auto& point = msg->points[i];
        // ROS_INFO("  Point %zu: x=%f, y=%f, z=%f", i, point.x, point.y, point.z);
        roadmap.mapBorders.add_point(point.x, point.y, point.z);
    }

    ROS_INFO_STREAM("[MapBorders] roadmap mapBorders: " << roadmap.mapBorders.to_string());
}

/* ====================================== Gates Processing ======================================== */
// This function read the message containing the gates data coming from the map package. 
// Extract this data and store the inside the Gates attributes of the give Roadmap element
void processGates(const geometry_msgs::PoseArray::ConstPtr& msg, Roadmap& roadmap) {
    // geometry_msgs/PoseArray
    // header: ... 
    // poses: 
    //  - 
    //   positions:
    //      x: ...
    //      y: ...
    //      z: ...
    //   orientation:
    //      x: ...
    //      y: ...
    //      z: ...
    //      w: ...

    ROS_INFO("[Gates] Received %zu gates:", msg->poses.size());

    if( msg->poses.size() <= 0){
        ROS_WARN("[Gates] No gates received");
        return; 
    }

    for (const auto& gate_msg : msg->poses) {
        Point position;
        position.x = gate_msg.position.x;  
        position.y = gate_msg.position.y;  
        position.z = gate_msg.position.z;  

        Orientation orientation;
        orientation.x = gate_msg.orientation.x;  
        orientation.y = gate_msg.orientation.y;  
        orientation.z = gate_msg.orientation.z;
        orientation.w = gate_msg.orientation.w;  

        // Add gate to roadmap
        if (!roadmap.gates.add_gate(position, orientation)) {
            ROS_WARN("[Gates] Failed to add gate at (%.2f, %.2f)", position.x, position.y);
        }
    }

    ROS_INFO_STREAM("[Gates] Roadmap gates: " << roadmap.gates.to_string());

}

/* ====================================== Victim Processing ======================================== */
// This function read the message containing the victims data coming from the map package. 
// Extract this data and store the inside the Victimes attributes of the give Roadmap element
void processVictims(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg, Roadmap& roadmap) {

    ROS_INFO("[Victims] Received %zu victims:", msg->obstacles.size());
    
    for (const auto& victim_msg : msg->obstacles) {
        // Check if polygon has points
        if (victim_msg.polygon.points.empty()) {
            ROS_WARN("[Victims] Victim has no center point, skipping");
            continue;
        }
        // Warn if more than one point
        if (victim_msg.polygon.points.size() > 1) {
            ROS_WARN("[Victims] Received %zu points, required only one as center", 
                     victim_msg.polygon.points.size());
        }
        
        // Get the center as the first point in the polygon
        Point center;
        center.x = victim_msg.polygon.points[0].x;  
        center.y = victim_msg.polygon.points[0].y;  
        center.z = victim_msg.polygon.points[0].z; 
        
        // Get radius from message
        float radius = victim_msg.radius;
        
        // Add victim to roadmap
        if (!roadmap.victims.add_victim(center, radius)) {
            ROS_WARN("[Victims] Failed to add victim at (%.2f, %.2f)", center.x, center.y);
        }
    }

    ROS_INFO_STREAM("[Victims] Roadmap victims: " << roadmap.victims.to_string());
}






/* ================================ ROADMAP INITIALIZATION =========================================== */
// This function initialize a Roadmap istance:
// 1) Get the map element from the map package: Borders, Obstacles and Victims
// 2) Popolate the roadmap with the nodes
void roadmap_init(ros::NodeHandle& nh)
{
    // Init the data structure for the Roadmap
    static Roadmap roadmap; 

    ROS_INFO("Initializing roadmap subscribers...");

    // STEP 1: INITILIZATION WITH THE MAP DATA
    // Get the map element: Borders, Gates, Obstacles and Victims from the map
    // NB: Since in our scenario the enviroment is static we sample only one time the map structure, obstacles and victims

    // Get map borders (once)
    auto map_borders_msg = ros::topic::waitForMessage<geometry_msgs::Polygon>(
        "/map_borders", nh);
    if (map_borders_msg) {
        processMapBorders(map_borders_msg, roadmap); 
    }

    // Get gates (once)
    auto gates_msg = ros::topic::waitForMessage<geometry_msgs::PoseArray>(
        "/gates", nh);
    if (map_borders_msg) {
        processGates(gates_msg, roadmap); 
    }

    // Get obstacles (once)
    auto obstacles_msg = ros::topic::waitForMessage<obstacles_msgs::ObstacleArrayMsg>(
        "/obstacles", nh);
    if (obstacles_msg) {
        processObstacles(obstacles_msg, roadmap); 
    }

    // Get map victims (once)
    auto victims_msg = ros::topic::waitForMessage<obstacles_msgs::ObstacleArrayMsg>(
        "/victims", nh);
    if (map_borders_msg) {
        processVictims(victims_msg, roadmap); 
    }

    /* TODO Get Limo Start Point
    auto start_msg = ros::topic::waitForMessage<obstacles_msgs::ObstacleArrayMsg>(
        "/??????", nh);
    if (map_borders_msg) {
        processStart(victims_msg, roadmap); 
    }*/

    // plot the roadmap
    roadmap.paint_roadmap();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roadmap_node");
    ros::NodeHandle nh;

    // Configura parametri robot
    double robot_radius = 0.25;
    double safety_margin = 0.05;
    Roadmap roadmap(robot_radius, safety_margin);

    // --- FASE 1: ACQUISIZIONE DATI (Bloccante) ---
    // ... (Il tuo codice esistente con waitForMessage) ...
    
    auto map_borders_msg = ros::topic::waitForMessage<geometry_msgs::Polygon>("/map_borders", nh);
    if (map_borders_msg) processMapBorders(map_borders_msg, roadmap);

    auto obstacles_msg = ros::topic::waitForMessage<obstacles_msgs::ObstacleArrayMsg>("/obstacles", nh);
    if (obstacles_msg) processObstacles(obstacles_msg, roadmap);
    
    // ... (altri wait for message: victims, gates) ...

    // --- FASE 2: PREPARAZIONE ALGORITMICA (INTEGRAZIONE) ---
    // Ora che abbiamo tutti i dati "human readable", creiamo la cache "machine readable"
    roadmap.update_collision_cache();

    // DA QUI IN POI PUOI USARE:
    // roadmap.is_state_valid(x, y)
    // roadmap.is_dubins_path_valid(...)
    // Per i tuoi algoritmi RRT/PRM

    ROS_INFO("Roadmap ready for planning.");

    // plot the roadmap (Opencv viz)
    roadmap.paint_roadmap();
    
    // Mantieni il nodo vivo se serve (o return 0 se finisce qui)
    // ros::spin(); 
    return 0;
}
