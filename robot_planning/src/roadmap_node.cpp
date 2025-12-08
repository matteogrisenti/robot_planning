// This file read the map border, obstacle and victims from the map_pck:
//  -   /obstacle
//  -   /map_borders
//  -   /victims
// Then it i

#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

#include "map/map_data_structures.h"
#include "combinatorial_planning/exact_cell_decomposition.h"
#include "combinatorial_planning/approximate_cell_decomposition.h"
#include "combinatorial_planning/maximum_clearance_roadmap.h"
#include "roadmap/roadmap_visualization.h"


/* ===================================== Obstacle Processing ======================================== */
// This function read the message containing the obstacles data coming from the map package. 
// Extract this data and store the inside the Obstacles attributes of the give Map element.
void processObstacles(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg, Map& map) {
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
        
        // Add obstacle to map
        map.obstacles.add_obstacle(&points, radius);
    }

    ROS_INFO_STREAM("[Obstacles] Map obstacles: " << map.obstacles.to_string());
}

/* ================================== Map Borders Processing ======================================== */
// This function read the message containing the map borders data coming from the map package. 
// Extract this data and store the inside the MapBoardes attributes of the give Map element.
void processMapBorders(const geometry_msgs::Polygon::ConstPtr& msg, Map& map){ 
    // geometry_msgs/Polygon
    // points:
    //      -
    //       x: ...
    //       y: ...
    //       z: ...

    ROS_INFO("[Borders] Received %zu vertices:", msg->points.size());

    for (size_t i = 0; i < msg->points.size(); ++i){
        const auto& point = msg->points[i];
        // ROS_INFO("  Point %zu: x=%f, y=%f, z=%f", i, point.x, point.y, point.z);
        map.borders.add_point(point.x, point.y, point.z);
    }

    ROS_INFO_STREAM("[Borders] Map borders: " << map.borders.to_string());
}


/* ====================================== Odometry Processing ======================================== */
// This function read the message containing the odometry data coming from the map package. 
// Extract this data and store the inside the Start attributes of the give Map element
void processOdometry(const nav_msgs::Odometry::ConstPtr& msg, Map& map) {
    ROS_INFO("[Odometry] Received robot position");
    
    // Extract position
    Point position;
    position.x = msg->pose.pose.position.x;
    position.y = msg->pose.pose.position.y;
    position.z = msg->pose.pose.position.z;
    
    // Extract orientation (quaternion)
    Orientation orientation;
    orientation.x = msg->pose.pose.orientation.x;
    orientation.y = msg->pose.pose.orientation.y;
    orientation.z = msg->pose.pose.orientation.z;
    orientation.w = msg->pose.pose.orientation.w;
    
    // Store as start position
    map.start = Start(position, orientation);
    
    ROS_INFO_STREAM("[Odometry] Start position set: " << map.start.to_string());
}

/* ====================================== Gates Processing ======================================== */
// This function read the message containing the gates data coming from the map package. 
// Extract this data and store the inside the Gates attributes of the give Map element
void processGates(const geometry_msgs::PoseArray::ConstPtr& msg, Map& map) {
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

        // Add gate to map
        if (!map.gates.add_gate(position, orientation)) {
            ROS_WARN("[Gates] Failed to add gate at (%.2f, %.2f)", position.x, position.y);
        }
    }

    ROS_INFO_STREAM("[Gates] Map gates: " << map.gates.to_string());

}

/* ====================================== Victim Processing ======================================== */
// This function read the message containing the victims data coming from the map package. 
// Extract this data and store the inside the Victimes attributes of the give Map element
void processVictims(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg, Map& map) {

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
        
        // Add victim to map
        if (!map.victims.add_victim(center, radius)) {
            ROS_WARN("[Victims] Failed to add victim at (%.2f, %.2f)", center.x, center.y);
        }
    }

    ROS_INFO_STREAM("[Victims] Map victims: " << map.victims.to_string());
}






/* ================================ ROADMAP INITIALIZATION =========================================== */
// This function initialize a Roadmap istance:
// 1) Get the map element from the map package: Borders, Obstacles and Victims
// 2) Popolate the roadmap with the nodes
void roadmap_init(ros::NodeHandle& nh)
{
    // Init the data structure for the map
    static Map map; 

    // STEP 1: INITILIZATION WITH THE MAP DATA
    // Get the map element: Borders, Gates, Obstacles and Victims from the map
    // NB: Since in our scenario the enviroment is static we sample only one time the map structure, obstacles and victims

    // Get map borders (once)
    auto map_borders_msg = ros::topic::waitForMessage<geometry_msgs::Polygon>("/map_borders", nh);
    if (map_borders_msg) {
        processMapBorders(map_borders_msg, map); 
    }

    // Get the start point: the odometry of the robot
    auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", nh);
    if (odom_msg) {
        processOdometry(odom_msg, map);
    }

    // Get gates (once)
    auto gates_msg = ros::topic::waitForMessage<geometry_msgs::PoseArray>("/gates", nh);
    if (map_borders_msg) {
        processGates(gates_msg, map); 
    }

    // Get obstacles (once)
    auto obstacles_msg = ros::topic::waitForMessage<obstacles_msgs::ObstacleArrayMsg>("/obstacles", nh);
    if (obstacles_msg) {
        processObstacles(obstacles_msg, map); 
    }

    // Get map victims (once)
    auto victims_msg = ros::topic::waitForMessage<obstacles_msgs::ObstacleArrayMsg>("/victims", nh);
    if (map_borders_msg) {
        processVictims(victims_msg, map); 
    }

    // plot the map
    map.paint_map();


    // initialize the Roadmap
    std::shared_ptr<Roadmap> roadmap; 

    // roadmap = ExactDecomposition::exactCellDecomposition(map);
    // roadmap = ApproximateDecomposition::approximateCellDecomposition(map, 5);
    roadmap = MaxClearanceRoadmap::maximumClearanceRoadmap(map);

    // Create visualizer
    roadmap_viz::RoadmapVizConfig config;
    config.vertex_radius = 10;  // Optional: customize
    roadmap_viz::RoadmapVisualizer viz(config);

    // Render
    viz.render(map, roadmap);

    // Display
    viz.display();  // Shows in window

    // Or save to file
    // viz.saveToFile("roadmap.png");
}





int main(int argc, char **argv)
{
    // Inizializza il nodo ROS
    ros::init(argc, argv, "roadmap_node");
    ros::NodeHandle nh;

    // Inizializza roadmap e i suoi subscriber
    roadmap_init(nh);

    // Loop principale per ricevere i messaggi
    ros::spin();

    return 0;
}
