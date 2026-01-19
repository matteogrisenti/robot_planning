#include "libraries/graph_search.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace GraphSearch {

    void rviz_plan(const std::vector<int>& path, const Roadmap& graph, const ros::Publisher& pub) {
        if (path.empty()) {
            ROS_WARN("rviz_plan called with empty path.");
            return;
        }

        // 1. Configure the Line Marker (The Edges)
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map"; // Assuming map frame
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "astar_path_edges";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;
        
        line_marker.scale.x = 0.05; // Line width

        // Color:
        line_marker.color.r = 0.0;
        line_marker.color.g = 200.0 / 255.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 0.5; // opacity

        // 2. Configure the Node Marker (The Vertices)
        visualization_msgs::Marker node_marker;
        node_marker.header.frame_id = "map";
        node_marker.header.stamp = ros::Time::now();
        node_marker.ns = "astar_path_nodes";
        node_marker.id = 1;
        node_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        node_marker.action = visualization_msgs::Marker::ADD;
        node_marker.pose.orientation.w = 1.0;

        node_marker.scale.x = 0.15; // Sphere diameter
        node_marker.scale.y = 0.15;
        node_marker.scale.z = 0.15;

        // Color
        node_marker.color.r = 0.0;
        node_marker.color.g = 1.0;
        node_marker.color.b = 1.0;
        node_marker.color.a = 1.0;

        // 3. Fill Geometry
        for (int nodeIdx : path) {
            // Retrieve vertex coordinates from the roadmap
            // Assuming your Roadmap class has a getVertex(int) method
            const Vertex& v = graph.getVertex(nodeIdx); 

            geometry_msgs::Point p;
            p.x = v.x;
            p.y = v.y;
            p.z = 0.0; // Assume 2D map, or v.z if available

            line_marker.points.push_back(p);
            node_marker.points.push_back(p);
        }

        // 4. Publish
        pub.publish(line_marker);
        pub.publish(node_marker);
    }
}