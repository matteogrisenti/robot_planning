#ifndef DUBINS_PLANNER_H
#define DUBINS_PLANNER_H

#include <ros/ros.h>
#include <robot_control/Reference.h>
#include <visualization_msgs/MarkerArray.h>

// Include your custom libraries
#include "dubins_planner/dubins_trajectory.h"
#include "dubins_planner/collision_checker.h"
#include "map_library/map_builder.h" 

class DubinsPlanner {
public:

    DubinsPlanner();
    /**
     * @brief Constructor
     * @param nh NodeHandle for publishers
     * @param robot_name Robot namespace
     * @param robot_radius Radius for collision checking (e.g., 0.25)
     * @param safety_margin Safety margin for collision checking (e.g., 0.05)
     */
    DubinsPlanner(ros::NodeHandle& nh, std::string robot_name, double robot_radius, double safety_margin);

    /**
     * @brief Updates the internal map for the collision checker.
     * Must be called at least once before planning.
     */
    void setMap(const Map& map);

    /**
     * @brief Plans a collision-free Dubins path from start to goal and stores it 
     * internally in current_curve_.
     * @param start_x Robot start X
     * @param start_y Robot start Y
     * @param start_th Robot start Theta
     * @param goal_x Goal X
     * @param goal_y Goal Y
     * @param goal_th Goal Theta
     * @param rho Minimum turning radius
     * @param debug_viz If true, publishes debug visualization markers
     * @return true if a valid, collision-free path was found
     */
    bool planPath(double start_x, double start_y, double start_th, 
                  double goal_x, double goal_y, double goal_th, 
                  double rho, bool debug_viz = false);

    /**
     * @brief Initializes execution state of the currently planned path.
     * @param velocity Desired linear velocity (m/s)
     */
    void startExecution(double velocity);

    /**
     * @brief Real executor of the planned path. It interpolate the path based on current robot pose.
     * to get the reference point to send to the robot. Reference is thne published inside this function
     * to the topic /robot_name/ref
     * @param curr_x Current robot X
     * @param curr_y Current robot Y
     * @param curr_th Current robot Theta
     * @return true if the path execution is finished
     */
    bool spin(double curr_x, double curr_y, double curr_th);

    /**
     * @brief Stops execution and sends a zero-velocity command.
     */
    void stop();

private:
    ros::NodeHandle nh_;  
    ros::Publisher pub_ref_; // For reference commands
    ros::Publisher pub_viz_; // For debug markers
    
    // Logic Objects
    Kinematics::DubinsSolver solver_;       // Persistent solver instance
    Kinematics::Trajectory current_path_;   // Updated trajectory storage
    CollisionChecker collision_checker_;    // Collision checker instance

    // Execution Flags
    bool map_received_;         // True if map has been set
    bool plan_valid_;           // True if the last plan is valid
    bool is_executing_;         // True if currently executing a plan
    // Execution Variables
    int current_segment_index_;     // Segment execution index
    double total_seg_distance_;     // Total length of the current path
    ros::Time start_time_;          // Start time of execution
    double target_velocity_;        // Target linear velocity

    // Helpers
    // Publishes the instantaneous reference point along the given arc at distance s_local
    void publishReference(const Kinematics::Segment& seg, double s_local);
    // Publishes debug visualization of the planned path
    void publishDebugViz(const Kinematics::Trajectory& curve, bool valid);
};

#endif // DUBINS_PLANNER_H