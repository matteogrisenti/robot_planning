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
     * @brief Plans a collision-free Dubins path from start to goal.
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
     * @brief Starts executing the currently planned path.
     * @param velocity Desired linear velocity (m/s)
     */
    void startExecution(double velocity);

    /**
     * @brief Main loop function. Call this at your control frequency (e.g., 50Hz).
     * Calculates the instantaneous reference point and publishes it.
     */
    bool spin();

    /**
     * @brief Stops execution and sends a zero-velocity command.
     */
    void stop();

private:
    ros::NodeHandle nh_;  
    ros::Publisher pub_ref_; // For reference commands
    ros::Publisher pub_viz_; // For debug markers

    // Logic Objects
    CollisionChecker collision_checker_;
    dubinscurve_out current_curve_;
    int current_best_idx_; // Needed for collision check verification

    // Execution State
    bool map_received_;
    bool plan_valid_;
    bool is_executing_;
    
    ros::Time start_time_;
    double target_velocity_;

    // Helpers
    // Publishes the instantaneous reference point along the given arc at distance s_local
    void publishReference(const dubinsarc_out& arc, double s_local);
    // Publishes debug visualization of the planned path
    void publishDebugViz(const dubinscurve_out& curve, bool valid);
};

#endif // DUBINS_PLANNER_H