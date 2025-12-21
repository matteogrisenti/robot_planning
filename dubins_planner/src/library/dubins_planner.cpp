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

// Initialize the current_curve_ and check for collisions
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

    // Reset the variable needed for identify the arc of the current cure during execution
    current_segment_index_ = 0; // Reset execution index
    total_arc_distance_ = 0.0; // Reset accumulated distance

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


// Initialize execution state 
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

// Stop execution
// TODO: Maybe need negative velocity to stop istanly, now it seem like go after the stop for inertia
void DubinsPlanner::stop() {
    is_executing_ = false;
    // Send zero-velocity command
    robot_control::Reference ref_msg;
    ref_msg.v_d = 0.0;
    ref_msg.omega_d = 0.0;
    ref_msg.plan_finished = true;
    pub_ref_.publish(ref_msg);
}

// Real Execution 
bool DubinsPlanner::spin(double curr_x, double curr_y, double curr_th) {
    if (!is_executing_) return false;

    // Get the current segment geometry
    dubinsarc_out* active_arc;
    if (current_segment_index_ == 0) active_arc = &current_curve_.a1;
    else if (current_segment_index_ == 1) active_arc = &current_curve_.a2;
    else active_arc = &current_curve_.a3;

    double s_local = 0;

    if (active_arc->k == 0) {
        // --- STRAIGHT LINE LOGIC ---
        // Project the robot onto the line to find progress
        s_local = sqrt(pow(curr_x - active_arc->x0, 2) + pow(curr_y - active_arc->y0, 2));
    } else {
        // --- CIRCLE/ARC LOGIC (Handles full curves) ---
        // 1. Find the center of the circle
        double R = 1.0 / active_arc->k;
        double center_x = active_arc->x0 - R * sin(active_arc->th0);
        double center_y = active_arc->y0 + R * cos(active_arc->th0);

        // 2. Calculate the angle from the center to the robot
        double angle_now = atan2(curr_y - center_y, curr_x - center_x);
        double angle_start = atan2(active_arc->y0 - center_y, active_arc->x0 - center_x);
        
        // 3. Calculate angular difference (careful with wrap-around!)
        double delta_angle = angle_now - angle_start;
        // Adjust for direction of rotation (k > 0 is Left, k < 0 is Right)
        if (active_arc->k > 0) { // Left turn
            while (delta_angle < 0) delta_angle += 2 * M_PI;
        } else { // Right turn
            while (delta_angle > 0) delta_angle -= 2 * M_PI;
        }
        
        s_local = fabs(delta_angle * R);
    }

    // --- SEGMENT SWITCHING ---
    // If we have finished the length of the current arc, move to the next
    if (s_local >= (active_arc->l - 0.1)) { // 10cm margin
        if (current_segment_index_ < 2) {
            current_segment_index_++;
            ROS_INFO("Switching to Segment %d", current_segment_index_);
            return false; // Keep spinning in the next frame
        } else {
            stop(); // Mission Complete
            return true; 
        }
    }

    // Publish the reference for the current progress
    publishReference(*active_arc, s_local);
    return false;
}

void DubinsPlanner::publishReference(const dubinsarc_out& arc, double s_local) {
    // Extract instantaneous point using 'circline' helper of dubins_trajectory.h
    long double x_ref, y_ref, th_ref;
    circline(s_local, arc.x0, arc.y0, arc.th0, arc.k, x_ref, y_ref, th_ref);

    // ENHANCED ERROR CHECKING with detailed diagnostics
    if (std::isnan(x_ref) || std::isnan(y_ref) || std::isnan(th_ref)) {
        ROS_ERROR("[DubinsPlanner] === NaN DETECTED IN REFERENCE ===");
        ROS_ERROR("[DubinsPlanner]   Segment: %d/3", current_segment_index_);
        ROS_ERROR("[DubinsPlanner]   Arc Start: (%.3Lf, %.3Lf, %.3Lf rad)", arc.x0, arc.y0, arc.th0);
        ROS_ERROR("[DubinsPlanner]   Arc Params: k=%.6Lf, length=%.3Lf m", arc.k, arc.l);
        ROS_ERROR("[DubinsPlanner]   Local Progress: s=%.3f / %.3Lf m", s_local, arc.l);
        ROS_ERROR("[DubinsPlanner]   Reference Output: (%.3Lf, %.3Lf, %.3Lf)", x_ref, y_ref, th_ref);
        ROS_ERROR("[DubinsPlanner] === POSSIBLE CAUSES ===");
        ROS_ERROR("[DubinsPlanner]   1. s_local exceeds arc.l (overshoot)");
        ROS_ERROR("[DubinsPlanner]   2. Invalid arc parameters from dubins_shortest_path");
        ROS_ERROR("[DubinsPlanner]   3. Numerical instability in circline() function");
        ROS_ERROR("[DubinsPlanner]   4. Robot far from path (localization error?)");
        
        stop(); // This sends 0 velocity and sets plan_finished = true
        return;
    }

    robot_control::Reference ref_msg;
    ref_msg.x_d = (double)x_ref;
    ref_msg.y_d = (double)y_ref;
    ref_msg.theta_d = (double)th_ref;
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