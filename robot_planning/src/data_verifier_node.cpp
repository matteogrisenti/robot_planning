#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <obstacles_msgs/ObstacleMsg.h>

class DataVerifier {
private:
    ros::NodeHandle nh_;
    
    // Subscribers
    ros::Subscriber obstacles_sub_;
    ros::Subscriber victims_sub_;
    ros::Subscriber gates_sub_;
    
    // Publishers for visualization
    ros::Publisher obstacles_markers_pub_;
    ros::Publisher victims_markers_pub_;
    ros::Publisher gates_markers_pub_;
    
    int obstacle_id_counter_;
    int victim_id_counter_;
    int gate_id_counter_;
    
public:
    DataVerifier() : obstacle_id_counter_(0), victim_id_counter_(0), gate_id_counter_(0) {
        // Subscribe to topics
        obstacles_sub_ = nh_.subscribe("/obstacles", 10, &DataVerifier::obstaclesCallback, this);
        victims_sub_ = nh_.subscribe("/victims", 10, &DataVerifier::victimsCallback, this);
        gates_sub_ = nh_.subscribe("/gates", 10, &DataVerifier::gatesCallback, this);
        
        // Publishers for verification markers
        obstacles_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/verification/obstacles", 10, true);
        victims_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/verification/victims", 10, true);
        gates_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/verification/gates", 10, true);
        
        ROS_INFO("Data Verifier Node Started!");
        ROS_INFO("Subscribing to: /obstacles, /victims, /gates");
        ROS_INFO("Publishing to: /verification/obstacles, /verification/victims, /verification/gates");
    }
    
    void obstaclesCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        ROS_INFO("Received %lu obstacles", msg->obstacles.size());
        
        visualization_msgs::MarkerArray markers;
        obstacle_id_counter_ = 0;
        
        for (const auto& obstacle : msg->obstacles) {
            // Check if it's a polygon or cylinder
            if (obstacle.polygon.points.size() > 1) {
                // Polygon obstacle
                ROS_INFO("  Obstacle %d: Polygon with %lu vertices", 
                         obstacle_id_counter_, obstacle.polygon.points.size());
                
                visualization_msgs::Marker line_strip;
                line_strip.header.frame_id = "map";
                line_strip.header.stamp = ros::Time::now();
                line_strip.ns = "obstacles_polygon";
                line_strip.id = obstacle_id_counter_++;
                line_strip.type = visualization_msgs::Marker::LINE_STRIP;
                line_strip.action = visualization_msgs::Marker::ADD;
                line_strip.scale.x = 0.05; // Line width
                line_strip.color.r = 1.0;
                line_strip.color.g = 0.5;
                line_strip.color.a = 1.0;
                
                for (const auto& point : obstacle.polygon.points) {
                    geometry_msgs::Point p;
                    p.x = point.x;
                    p.y = point.y;
                    p.z = 0.1;
                    line_strip.points.push_back(p);
                }
                // Close the polygon
                if (!obstacle.polygon.points.empty()) {
                    geometry_msgs::Point p;
                    p.x = obstacle.polygon.points[0].x;
                    p.y = obstacle.polygon.points[0].y;
                    p.z = 0.1;
                    line_strip.points.push_back(p);
                }
                
                markers.markers.push_back(line_strip);
                
            } else if (obstacle.polygon.points.size() == 1 && obstacle.radius > 0) {
                // Cylinder obstacle
                ROS_INFO("  Obstacle %d: Cylinder at (%.2f, %.2f) radius %.2f", 
                         obstacle_id_counter_,
                         obstacle.polygon.points[0].x,
                         obstacle.polygon.points[0].y,
                         obstacle.radius);
                
                visualization_msgs::Marker cylinder;
                cylinder.header.frame_id = "map";
                cylinder.header.stamp = ros::Time::now();
                cylinder.ns = "obstacles_cylinder";
                cylinder.id = obstacle_id_counter_++;
                cylinder.type = visualization_msgs::Marker::CYLINDER;
                cylinder.action = visualization_msgs::Marker::ADD;
                cylinder.pose.position.x = obstacle.polygon.points[0].x;
                cylinder.pose.position.y = obstacle.polygon.points[0].y;
                cylinder.pose.position.z = 0.5;
                cylinder.scale.x = obstacle.radius * 2;
                cylinder.scale.y = obstacle.radius * 2;
                cylinder.scale.z = 1.0;
                cylinder.color.r = 1.0;
                cylinder.color.g = 0.5;
                cylinder.color.a = 0.7;
                
                markers.markers.push_back(cylinder);
            }
        }
        
        obstacles_markers_pub_.publish(markers);
    }
    
    void victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        ROS_INFO("Received %lu victims", msg->obstacles.size());
        
        visualization_msgs::MarkerArray markers;
        victim_id_counter_ = 0;
        
        for (const auto& victim : msg->obstacles) {
            if (victim.polygon.points.size() == 1) {
                double weight = victim.radius;
                double x = victim.polygon.points[0].x;
                double y = victim.polygon.points[0].y;
                
                ROS_INFO("  Victim %d: at (%.2f, %.2f) weight %.0f", 
                         victim_id_counter_, x, y, weight);
                
                // Create circle with radius 0.5m as specified
                visualization_msgs::Marker circle;
                circle.header.frame_id = "map";
                circle.header.stamp = ros::Time::now();
                circle.ns = "victims_circles";
                circle.id = victim_id_counter_;
                circle.type = visualization_msgs::Marker::CYLINDER;
                circle.action = visualization_msgs::Marker::ADD;
                circle.pose.position.x = x;
                circle.pose.position.y = y;
                circle.pose.position.z = 0.05;
                circle.scale.x = 1.0; // Diameter = 2 * 0.5m radius
                circle.scale.y = 1.0;
                circle.scale.z = 0.1;
                circle.color.r = 0.0;
                circle.color.g = 1.0;
                circle.color.b = 0.0;
                circle.color.a = 0.5;
                
                markers.markers.push_back(circle);
                
                // Add text for weight
                visualization_msgs::Marker text;
                text.header.frame_id = "map";
                text.header.stamp = ros::Time::now();
                text.ns = "victims_weight";
                text.id = victim_id_counter_;
                text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text.action = visualization_msgs::Marker::ADD;
                text.pose.position.x = x;
                text.pose.position.y = y;
                text.pose.position.z = 0.5;
                text.scale.z = 0.3;
                text.color.r = 1.0;
                text.color.g = 1.0;
                text.color.b = 1.0;
                text.color.a = 1.0;
                text.text = std::to_string(static_cast<int>(weight));
                
                markers.markers.push_back(text);
                
                victim_id_counter_++;
            }
        }
        
        victims_markers_pub_.publish(markers);
    }
    
    void gatesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
        ROS_INFO("Received %lu gates", msg->poses.size());
        
        visualization_msgs::MarkerArray markers;
        gate_id_counter_ = 0;
        
        for (const auto& pose : msg->poses) {
            ROS_INFO("  Gate %d: at (%.2f, %.2f)", 
                     gate_id_counter_, pose.position.x, pose.position.y);
            
            visualization_msgs::Marker arrow;
            arrow.header.frame_id = "map";
            arrow.header.stamp = ros::Time::now();
            arrow.ns = "gates_arrows";
            arrow.id = gate_id_counter_++;
            arrow.type = visualization_msgs::Marker::ARROW;
            arrow.action = visualization_msgs::Marker::ADD;
            arrow.pose = pose;
            arrow.scale.x = 1.0; // Arrow length
            arrow.scale.y = 0.1; // Arrow width
            arrow.scale.z = 0.1; // Arrow height
            arrow.color.r = 1.0;
            arrow.color.g = 0.0;
            arrow.color.b = 1.0; // Magenta for verification
            arrow.color.a = 1.0;
            
            markers.markers.push_back(arrow);
        }
        
        gates_markers_pub_.publish(markers);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_verifier");
    DataVerifier verifier;
    ros::spin();
    return 0;
}
