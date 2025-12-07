#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

// --- INTEGRAZIONE LIBRERIE ---
// Note: dubins_trajectory.h è già incluso da data_structures.h
#include "roadmap/data_structures.h"

// ============================================================================
// DEFINIZIONE VARIABILI GLOBALI (Richieste dalla libreria dubins C-Style)
// ============================================================================
bool DEBUG = false;
long double X0, Y0, Th0, Xf, Yf, Thf, Kmax;
int pidx;
int no_waypts, step, no_of_samples;
long double angle_step;
dubinscurve_out dubin_curve;
point init_pt, final_pt;
std::vector<point> best_path;
// ============================================================================

class DataVerifier {
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_map_borders;
    ros::Subscriber sub_gates;
    ros::Subscriber sub_obs;
    ros::Subscriber sub_victims;
    ros::Publisher pub_markers;

    // Roadmap integrata
    Roadmap roadmap;
    bool roadmap_ready = false;
    bool borders_received = false;
    bool gates_received = false;
    bool obstacles_received = false;
    bool victims_received = false;
    
    Point goal_position;
    double goal_theta = 0.0;
    bool goal_available = false;

public:
    DataVerifier() : roadmap(0.25, 0.05) {  // robot_radius=0.25m, safety_margin=0.05m
        pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/debug/environment_markers", 1, true);
        
        // Subscribe ai topic della mappa
        sub_map_borders = nh.subscribe("/map_borders", 1, &DataVerifier::mapBordersCallback, this);
        sub_gates = nh.subscribe("/gates", 1, &DataVerifier::gatesCallback, this);
        sub_obs = nh.subscribe("/obstacles", 1, &DataVerifier::obstaclesCallback, this);
        sub_victims = nh.subscribe("/victims", 1, &DataVerifier::victimsCallback, this);
        
        ROS_INFO("=== DATA VERIFIER WITH ROADMAP + DUBINS + COLLISION CHECKER ===");
    }

    // ========== CALLBACKS PER COSTRUIRE ROADMAP ==========
    
    void mapBordersCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
        if (borders_received) return;
        borders_received = true;
        
        ROS_INFO("[MapBorders] Received %zu vertices", msg->points.size());
        for (const auto& point : msg->points) {
            roadmap.mapBorders.add_point(point.x, point.y, point.z);
        }
        checkAndInitializeRoadmap();
    }

    void gatesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        if (gates_received) return;
        gates_received = true;
        
        ROS_INFO("[Gates] Received %zu gates", msg->poses.size());
        for (const auto& gate_msg : msg->poses) {
            Point position = {gate_msg.position.x, gate_msg.position.y, gate_msg.position.z};
            Orientation orientation = {gate_msg.orientation.x, gate_msg.orientation.y, 
                                      gate_msg.orientation.z, gate_msg.orientation.w};
            roadmap.gates.add_gate(position, orientation);
            
            // Salva primo gate come goal
            if (!goal_available) {
                goal_position = position;
                // Calcola yaw da quaternione
                goal_theta = std::atan2(2.0 * (gate_msg.orientation.w * gate_msg.orientation.z + 
                                               gate_msg.orientation.x * gate_msg.orientation.y),
                                       1.0 - 2.0 * (gate_msg.orientation.y * gate_msg.orientation.y + 
                                                   gate_msg.orientation.z * gate_msg.orientation.z));
                goal_available = true;
                ROS_INFO("[Goal] Set to gate at (%.2f, %.2f, theta=%.2f)", 
                         goal_position.x, goal_position.y, goal_theta);
            }
        }
        checkAndInitializeRoadmap();
    }

    void obstaclesCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        if (obstacles_received) return;
        obstacles_received = true;
        
        ROS_INFO("[Obstacles] Received %zu obstacles", msg->obstacles.size());
        for (const auto& obstacle_msg : msg->obstacles) {
            std::vector<Point> points;
            for (const auto& point_msg : obstacle_msg.polygon.points) {
                points.push_back({point_msg.x, point_msg.y, point_msg.z});
            }
            roadmap.obstacles.add_obstacle(&points, obstacle_msg.radius);
        }
        checkAndInitializeRoadmap();
    }

    void victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        if (victims_received) return;
        victims_received = true;
        
        ROS_INFO("[Victims] Received %zu victims", msg->obstacles.size());
        for (const auto& victim_msg : msg->obstacles) {
            if (victim_msg.polygon.points.empty()) continue;
            Point center = {victim_msg.polygon.points[0].x, victim_msg.polygon.points[0].y, 
                          victim_msg.polygon.points[0].z};
            roadmap.victims.add_victim(center, victim_msg.radius);
        }
        checkAndInitializeRoadmap();
    }

    void checkAndInitializeRoadmap() {
        if (roadmap_ready) return;
        if (!borders_received || !gates_received || !obstacles_received || !victims_received) return;
        
        ROS_INFO("[Roadmap] All data received, initializing collision cache...");
        roadmap.update_collision_cache();
        roadmap_ready = true;
        ROS_INFO("[Roadmap] READY for planning!");
    }

    // ========== PATH PLANNING E VISUALIZZAZIONE ==========

    void sampleAndAddToMarker(dubinsarc_out* arc, visualization_msgs::Marker& marker) {
        double step_size = 0.1;
        int steps = std::ceil(arc->l / step_size);
        
        for (int i = 0; i <= steps; ++i) {
            double s = std::min(i * step_size, (double)arc->l);
            long double x, y, th;
            circline(s, arc->x0, arc->y0, arc->th0, arc->k, x, y, th);
            
            geometry_msgs::Point p;
            p.x = (double)x; p.y = (double)y; p.z = 0.0;
            marker.points.push_back(p);
        }
    }

    void planAndVisualize() {
        if (!roadmap_ready || !goal_available) return;

        // Start position (origine)
        Point start = {0.0, 0.0, 0.0};
        double start_theta = 0.0;
        double turning_radius = 0.5;
        double k_max = 1.0 / turning_radius;

        // Calcola Dubins path
        int best_primitive_idx = -1;
        dubinscurve_out result_curve;
        dubins_shortest_path(start.x, start.y, start_theta, 
                            goal_position.x, goal_position.y, goal_theta, 
                            k_max, best_primitive_idx, &result_curve);

        if (best_primitive_idx < 0) {
            ROS_WARN_THROTTLE(5, "No valid Dubins path found!");
            return;
        }

        // Valida con collision checker
        bool path_valid = roadmap.is_dubins_path_valid(start, start_theta, 
                                                       goal_position, goal_theta, 
                                                       turning_radius);

        ROS_INFO_THROTTLE(5, "Path: Length=%.2f, Valid=%s", 
                         (double)result_curve.L, path_valid ? "YES" : "NO");

        // Visualizza tutto
        publishRoadmapMarkers();
        publishDubinsPath(result_curve, path_valid);
    }

    void publishRoadmapMarkers() {
        visualization_msgs::MarkerArray marker_array;

        // Bordi mappa (LINE_STRIP nero)
        visualization_msgs::Marker borders;
        borders.header.frame_id = "map";
        borders.header.stamp = ros::Time::now();
        borders.ns = "map_borders";
        borders.id = 2000;
        borders.type = visualization_msgs::Marker::LINE_STRIP;
        borders.action = visualization_msgs::Marker::ADD;
        borders.scale.x = 0.1;
        borders.color.r = 0.0; borders.color.g = 0.0; borders.color.b = 0.0; borders.color.a = 1.0;
        
        const auto& border_pts = roadmap.mapBorders.get_points();
        for (const auto& p : border_pts) {
            geometry_msgs::Point pt;
            pt.x = p.x; pt.y = p.y; pt.z = 0.0;
            borders.points.push_back(pt);
        }
        if (!border_pts.empty()) {
            borders.points.push_back(borders.points[0]); // Chiudi poligono
            marker_array.markers.push_back(borders);
        }

        // Ostacoli (CUBE_LIST rosso)
        int obs_id = 3000;
        const auto& obstacles = roadmap.obstacles.get_obstacles();
        for (const auto& obs : obstacles) {
            const auto& pts = obs.get_points();
            if (pts.size() < 3) continue;
            
            visualization_msgs::Marker obs_marker;
            obs_marker.header.frame_id = "map";
            obs_marker.header.stamp = ros::Time::now();
            obs_marker.ns = "obstacles";
            obs_marker.id = obs_id++;
            obs_marker.type = visualization_msgs::Marker::LINE_STRIP;
            obs_marker.action = visualization_msgs::Marker::ADD;
            obs_marker.scale.x = 0.08;
            obs_marker.color.r = 1.0; obs_marker.color.g = 0.0; obs_marker.color.b = 0.0; obs_marker.color.a = 0.8;
            
            for (const auto& p : pts) {
                geometry_msgs::Point pt;
                pt.x = p.x; pt.y = p.y; pt.z = 0.0;
                obs_marker.points.push_back(pt);
            }
            obs_marker.points.push_back(obs_marker.points[0]); // Chiudi
            marker_array.markers.push_back(obs_marker);
        }

        // Vittime (SPHERE verde)
        int victim_id = 1000;
        const auto& victims = roadmap.victims.get_victims();
        for (const auto& v : victims) {
            visualization_msgs::Marker sphere;
            sphere.header.frame_id = "map";
            sphere.header.stamp = ros::Time::now();
            sphere.ns = "victims";
            sphere.id = victim_id++;
            sphere.type = visualization_msgs::Marker::SPHERE;
            sphere.action = visualization_msgs::Marker::ADD;
            sphere.pose.position.x = v.get_center().x;
            sphere.pose.position.y = v.get_center().y;
            sphere.pose.position.z = 0.0;
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = 0.3; sphere.scale.y = 0.3; sphere.scale.z = 0.1;
            sphere.color.r = 0.0; sphere.color.g = 1.0; sphere.color.b = 0.0; sphere.color.a = 0.7;
            marker_array.markers.push_back(sphere);
        }

        // Gate (ARROW blu)
        int gate_id = 4000;
        const auto& gates = roadmap.gates.get_gates();
        for (const auto& g : gates) {
            visualization_msgs::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = ros::Time::now();
            arrow.ns = "gates";
            arrow.id = gate_id++;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;
            
            const auto& pos = g.get_position();
            const auto& ori = g.get_orientation();
            arrow.pose.position.x = pos.x;
            arrow.pose.position.y = pos.y;
            arrow.pose.position.z = 0.5;
            arrow.pose.orientation.x = ori.x;
            arrow.pose.orientation.y = ori.y;
            arrow.pose.orientation.z = ori.z;
            arrow.pose.orientation.w = ori.w;
            arrow.scale.x = 1.0; arrow.scale.y = 0.2; arrow.scale.z = 0.2;
            arrow.color.r = 0.0; arrow.color.g = 0.0; arrow.color.b = 1.0; arrow.color.a = 1.0;
            marker_array.markers.push_back(arrow);
        }

        pub_markers.publish(marker_array);
    }

    void publishDubinsPath(const dubinscurve_out& curve, bool valid) {
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "dubins_path";
        path_marker.id = 5000;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.scale.x = valid ? 0.08 : 0.05;
        
        if (valid) {
            path_marker.color.r = 0.0; path_marker.color.g = 1.0; path_marker.color.b = 0.0; 
            path_marker.color.a = 1.0;
        } else {
            path_marker.color.r = 1.0; path_marker.color.g = 0.0; path_marker.color.b = 0.0; 
            path_marker.color.a = 0.5;
        }

        dubinscurve_out curve_copy = curve;
        sampleAndAddToMarker(&curve_copy.a1, path_marker);
        sampleAndAddToMarker(&curve_copy.a2, path_marker);
        sampleAndAddToMarker(&curve_copy.a3, path_marker);

        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(path_marker);
        pub_markers.publish(marker_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_verifier");
    DataVerifier verifier;
    
    ros::Rate r(2); // 2 Hz
    while(ros::ok()) {
        verifier.planAndVisualize();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}