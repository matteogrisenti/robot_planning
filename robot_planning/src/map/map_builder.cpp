#include "map/map_builder.h"
#include <stdexcept>

namespace map_builder {

MapBuilder::MapBuilder(ros::NodeHandle& nh, double timeout)
    : nh_(nh), timeout_(timeout) {}

Map MapBuilder::buildMap() {
    return buildMap("/map_borders", "/odom", "/gates", "/obstacles", "/victims");
}

Map MapBuilder::buildMap(const std::string& borders_topic,
                         const std::string& odom_topic,
                         const std::string& gates_topic,
                         const std::string& obstacles_topic,
                         const std::string& victims_topic) {
    Map map;
    
    ROS_INFO("[MapBuilder] Starting map construction...");
    
    // Get map borders
    ROS_INFO("[MapBuilder] Waiting for map borders on topic: %s", borders_topic.c_str());
    auto map_borders_msg = ros::topic::waitForMessage<geometry_msgs::Polygon>(
        borders_topic, nh_, ros::Duration(timeout_));
    if (!map_borders_msg) {
        throw std::runtime_error("Failed to receive map borders within timeout");
    }
    processMapBorders(map_borders_msg, map);
    
    // Get odometry (start position)
    ROS_INFO("[MapBuilder] Waiting for odometry on topic: %s", odom_topic.c_str());
    auto odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>(
        odom_topic, nh_, ros::Duration(timeout_));
    if (!odom_msg) {
        throw std::runtime_error("Failed to receive odometry within timeout");
    }
    processOdometry(odom_msg, map);
    
    // Get gates
    ROS_INFO("[MapBuilder] Waiting for gates on topic: %s", gates_topic.c_str());
    auto gates_msg = ros::topic::waitForMessage<geometry_msgs::PoseArray>(
        gates_topic, nh_, ros::Duration(timeout_));
    if (!gates_msg) {
        ROS_WARN("[MapBuilder] Failed to receive gates within timeout");
    } else {
        processGates(gates_msg, map);
    }
    
    // Get obstacles
    ROS_INFO("[MapBuilder] Waiting for obstacles on topic: %s", obstacles_topic.c_str());
    auto obstacles_msg = ros::topic::waitForMessage<obstacles_msgs::ObstacleArrayMsg>(
        obstacles_topic, nh_, ros::Duration(timeout_));
    if (!obstacles_msg) {
        ROS_WARN("[MapBuilder] Failed to receive obstacles within timeout");
    } else {
        processObstacles(obstacles_msg, map);
    }
    
    // Get victims
    ROS_INFO("[MapBuilder] Waiting for victims on topic: %s", victims_topic.c_str());
    auto victims_msg = ros::topic::waitForMessage<obstacles_msgs::ObstacleArrayMsg>(
        victims_topic, nh_, ros::Duration(timeout_));
    if (!victims_msg) {
        ROS_WARN("[MapBuilder] Failed to receive victims within timeout");
    } else {
        processVictims(victims_msg, map);
    }
    
    ROS_INFO("[MapBuilder] Map construction complete");
    
    return map;
}

void MapBuilder::processObstacles(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg, Map& map) {
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

    ROS_INFO("[MapBuilder::Obstacles] Received %zu obstacles", msg->obstacles.size());
    
    for (const auto& obstacle_msg : msg->obstacles) {
        std::vector<Point> points;
        points.reserve(obstacle_msg.polygon.points.size());
        
        for (const auto& point_msg : obstacle_msg.polygon.points) {
            points.push_back({point_msg.x, point_msg.y, point_msg.z});
        }
        
        float radius = obstacle_msg.radius;
        map.obstacles.add_obstacle(&points, radius);
    }
    
    ROS_INFO_STREAM("[MapBuilder::Obstacles] " << map.obstacles.to_string());
}

void MapBuilder::processMapBorders(const geometry_msgs::Polygon::ConstPtr& msg, Map& map) {
    // geometry_msgs/Polygon
    // points:
    //      -
    //       x: ...
    //       y: ...
    //       z: ...

    ROS_INFO("[MapBuilder::Borders] Received %zu vertices", msg->points.size());
    
    for (const auto& point : msg->points) {
        map.borders.add_point(point.x, point.y, point.z);
    }
    
    ROS_INFO_STREAM("[MapBuilder::Borders] " << map.borders.to_string());
}

void MapBuilder::processOdometry(const nav_msgs::Odometry::ConstPtr& msg, Map& map) {
    ROS_INFO("[MapBuilder::Odometry] Received robot position");
    
    Point position;
    position.x = msg->pose.pose.position.x;
    position.y = msg->pose.pose.position.y;
    position.z = msg->pose.pose.position.z;
    
    Orientation orientation;
    orientation.x = msg->pose.pose.orientation.x;
    orientation.y = msg->pose.pose.orientation.y;
    orientation.z = msg->pose.pose.orientation.z;
    orientation.w = msg->pose.pose.orientation.w;
    
    map.start = Start(position, orientation);
    
    ROS_INFO_STREAM("[MapBuilder::Odometry] " << map.start.to_string());
}

void MapBuilder::processGates(const geometry_msgs::PoseArray::ConstPtr& msg, Map& map) {
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

    ROS_INFO("[MapBuilder::Gates] Received %zu gates", msg->poses.size());
    
    if (msg->poses.size() <= 0) {
        ROS_WARN("[MapBuilder::Gates] No gates received");
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
        
        if (!map.gates.add_gate(position, orientation)) {
            ROS_WARN("[MapBuilder::Gates] Failed to add gate at (%.2f, %.2f)", 
                     position.x, position.y);
        }
    }
    
    ROS_INFO_STREAM("[MapBuilder::Gates] " << map.gates.to_string());
}

void MapBuilder::processVictims(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg, Map& map) {
    ROS_INFO("[MapBuilder::Victims] Received %zu victims", msg->obstacles.size());
    
    for (const auto& victim_msg : msg->obstacles) {
        if (victim_msg.polygon.points.empty()) {
            ROS_WARN("[MapBuilder::Victims] Victim has no center point, skipping");
            continue;
        }
        
        if (victim_msg.polygon.points.size() > 1) {
            ROS_WARN("[MapBuilder::Victims] Received %zu points, expected only one as center",
                     victim_msg.polygon.points.size());
        }
        
        Point center;
        center.x = victim_msg.polygon.points[0].x;
        center.y = victim_msg.polygon.points[0].y;
        center.z = victim_msg.polygon.points[0].z;
        
        float radius = victim_msg.radius;
        
        if (!map.victims.add_victim(center, radius)) {
            ROS_WARN("[MapBuilder::Victims] Failed to add victim at (%.2f, %.2f)",
                     center.x, center.y);
        }
    }
    
    ROS_INFO_STREAM("[MapBuilder::Victims] " << map.victims.to_string());
}

} // namespace map_builder