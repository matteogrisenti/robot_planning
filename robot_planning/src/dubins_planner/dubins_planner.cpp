#include "dubins_planner/dubins_planner.h"

DubinsPlanner::DubinsPlanner()
    : nh_("~"), 
      plan_valid_(false),
      is_executing_(false)
{
    // Default constructor (no publishers)
}

DubinsPlanner::DubinsPlanner(ros::NodeHandle& nh, std::string robot_name, double robot_radius, double safety_margin)
    : nh_(nh), 
      collision_checker_(robot_radius, safety_margin),
      map_received_(false),
      plan_valid_(false),
      is_executing_(false)
{
    // Publishers
    pub_ref_ = nh_.advertise<robot_control::Reference>("/" + robot_name + "/ref", 1);
    pub_viz_ = nh_.advertise<visualization_msgs::MarkerArray>("/" + robot_name + "/planner_debug", 1);
}

void DubinsPlanner::setMap(const Map& map) {
    collision_checker_.update_collision_cache(map);
    map_received_ = true;
    ROS_INFO("[DubinsPlanner] Map updated.");
}

bool DubinsPlanner::planPath(double start_x, double start_y, double start_th, 
                             double goal_x, double goal_y, double goal_th, 
                             double rho, bool debug_viz) 
{
    if (!map_received_) {
        ROS_WARN("[DubinsPlanner] Cannot plan: No map received.");
        return false;
    }

    // 1. Compute Dubins Curve (Geometric)
    // using 1.0/rho as Kmax
    dubins_shortest_path(start_x, start_y, start_th, 
                         goal_x, goal_y, goal_th, 
                         1.0/rho, current_best_idx_, &current_curve_);

    plan_valid_ = false;

    // 2. Check if geometric solution exists
    if (current_best_idx_ >= 0) {
        // 3. Check Collisions
        if (collision_checker_.is_dubins_path_valid(current_best_idx_, current_curve_)) {
            plan_valid_ = true;
            ROS_INFO("[DubinsPlanner] Path Found! Length: %.2Lf m", current_curve_.L);
        } else {
            ROS_WARN("[DubinsPlanner] Path found but collides with obstacles.");
        }
    } else {
        ROS_WARN("[DubinsPlanner] No geometric Dubins path found.");
    }

    // Debug Visualization
    if (debug_viz) {
        publishDebugViz(current_curve_, plan_valid_);
    }
    
    return plan_valid_;
}

void DubinsPlanner::startExecution(double velocity) {
    if (!plan_valid_) {
        ROS_ERROR("[DubinsPlanner] Cannot execute: No valid plan.");
        return;
    }
    
    target_velocity_ = velocity;
    start_time_ = ros::Time::now();
    is_executing_ = true;
    ROS_INFO("[DubinsPlanner] Execution started at %.2f m/s", velocity);
}

void DubinsPlanner::stop() {
    is_executing_ = false;
    // Send zero-velocity command
    robot_control::Reference ref_msg;
    ref_msg.v_d = 0.0;
    ref_msg.omega_d = 0.0;
    ref_msg.plan_finished = true;
    pub_ref_.publish(ref_msg);
}

void DubinsPlanner::spin() {
    if (!is_executing_) return;

    // 1. Calculate progress
    double time_elapsed = (ros::Time::now() - start_time_).toSec();
    double dist = time_elapsed * target_velocity_;

    // 2. Cast lengths to double for comparison
    double l1 = (double)current_curve_.a1.l;
    double l2 = (double)current_curve_.a2.l;
    double l3 = (double)current_curve_.a3.l;
    double total_L = (double)current_curve_.L;

    // 3. Select Segment
    if (dist < l1) {
        publishReference(current_curve_.a1, dist);
    } else if (dist < (l1 + l2)) {
        publishReference(current_curve_.a2, dist - l1);
    } else if (dist < total_L) {
        publishReference(current_curve_.a3, dist - (l1 + l2));
    } else {
        ROS_INFO("[DubinsPlanner] Goal Reached.");
        stop();
    }
}

void DubinsPlanner::publishReference(const dubinsarc_out& arc, double s_local) {
    // Extract instantaneous point using 'circline' helper of dubins_trajectory.h
    long double x, y, th;
    circline(s_local, arc.x0, arc.y0, arc.th0, arc.k, x, y, th);

    robot_control::Reference ref_msg;
    ref_msg.x_d = (double)x;
    ref_msg.y_d = (double)y;
    ref_msg.theta_d = (double)th;
    ref_msg.v_d = target_velocity_;
    ref_msg.omega_d = (double)arc.k * target_velocity_;
    ref_msg.plan_finished = false;

    pub_ref_.publish(ref_msg);
}

void DubinsPlanner::publishDebugViz(const dubinscurve_out& curve, bool valid) {
    // Helper to visualize the planned path in Rviz (Red = Invalid, Green = Valid)
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.header.frame_id = "map"; // Or "odom" depending on your system
    m.header.stamp = ros::Time::now();
    m.ns = "dubins_plan";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.05;
    m.pose.orientation.w = 1.0;
    
    if (valid) { m.color.g = 1.0; m.color.a = 1.0; } 
    else       { m.color.r = 1.0; m.color.a = 1.0; }

    auto add_arc_points = [&](const dubinsarc_out& arc) {
        double step = 0.1;
        int n = std::ceil(arc.l / step);
        for(int i=0; i<=n; i++) {
            double s = std::min((double)arc.l, i*step);
            long double x, y, th;
            circline(s, arc.x0, arc.y0, arc.th0, arc.k, x, y, th);
            geometry_msgs::Point p; p.x = x; p.y = y;
            m.points.push_back(p);
        }
    };

    if (current_best_idx_ >= 0) {
        add_arc_points(curve.a1);
        add_arc_points(curve.a2);
        add_arc_points(curve.a3);
    }
    
    ma.markers.push_back(m);
    pub_viz_.publish(ma);
}