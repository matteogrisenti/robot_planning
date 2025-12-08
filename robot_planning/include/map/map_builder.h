#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H

#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>

#include "map/map_data_structures.h"


namespace map_builder {

/**
 * @brief Builds a Map object from ROS messages
 * 
 * This class encapsulates all logic for retrieving map data from ROS topics
 * and populating a Map data structure. It handles borders, obstacles, victims,
 * gates, and the robot's starting position.
 */
class MapBuilder {
public:
    /**
     * @brief Construct a MapBuilder
     * @param nh ROS NodeHandle for topic communication
     * @param timeout Timeout for waiting on topics (default: 10 seconds)
     */
    explicit MapBuilder(ros::NodeHandle& nh, double timeout = 10.0);

    /**
     * @brief Build the complete map from ROS topics
     * @return Map object populated with all available data
     * @throws std::runtime_error if required topics are not available
     */
    Map buildMap();

    /**
     * @brief Build map with specific topic names
     * @param borders_topic Topic name for map borders
     * @param odom_topic Topic name for odometry
     * @param gates_topic Topic name for gates
     * @param obstacles_topic Topic name for obstacles
     * @param victims_topic Topic name for victims
     * @return Map object populated with data
     */
    Map buildMap(const std::string& borders_topic,
                 const std::string& odom_topic,
                 const std::string& gates_topic,
                 const std::string& obstacles_topic,
                 const std::string& victims_topic);

private:
    ros::NodeHandle& nh_;
    double timeout_;

    // Processing functions for each message type
    void processObstacles(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg, Map& map);
    void processMapBorders(const geometry_msgs::Polygon::ConstPtr& msg, Map& map);
    void processOdometry(const nav_msgs::Odometry::ConstPtr& msg, Map& map);
    void processGates(const geometry_msgs::PoseArray::ConstPtr& msg, Map& map);
    void processVictims(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg, Map& map);
};

} // namespace map_builder

#endif // MAP_BUILDER_H