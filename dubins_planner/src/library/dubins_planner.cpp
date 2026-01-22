#include "dubins_planner/dubins_planner.h"
#include <algorithm> 

DubinsPlanner::DubinsPlanner()
    : nh_("~"), 
      solver_(1.0),
      plan_valid_(false),
      is_executing_(false)
{
    // Default constructor (no publishers)
}

DubinsPlanner::DubinsPlanner(ros::NodeHandle& nh, std::string robot_name, double robot_radius, double safety_margin)
    : nh_(nh), 
      solver_(1.0),
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

    // Set up start and goal states
    Kinematics::State start = {start_x, start_y, start_th};
    Kinematics::State goal = {goal_x, goal_y, goal_th};
    ROS_INFO("[DubinsPlanner] Planning from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f) with rho=%.2f", 
             start_x, start_y, start_th, goal_x, goal_y, goal_th, rho);

    // Re-initialize solver with current turning radius
    solver_ = Kinematics::DubinsSolver(1.0 / rho);

   
    std::vector<Kinematics::Trajectory> candidates;
    solver_.compute_candidates(start, goal, 6, candidates);

    if (candidates.empty()) {
        ROS_WARN("[DubinsPlanner] No kinematic path found.");
        return false;
    }

    current_segment_index_ = 0;     // Reset execution index
    plan_valid_ = false;            // Reset plan validity

    std::sort(candidates.begin(), candidates.end(), [](const Kinematics::Trajectory& a, const Kinematics::Trajectory& b) {

        bool a_is_loop = (a.type == Kinematics::PathType::RLR || a.type == Kinematics::PathType::LRL);
        bool b_is_loop = (b.type == Kinematics::PathType::RLR || b.type == Kinematics::PathType::LRL);
        
        if (a_is_loop != b_is_loop) return !a_is_loop;
        
        return a.total_length < b.total_length;
    });

    // --- Iterate through sorted candidates ---
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& traj = candidates[i];
        
        // Check collision for this specific trajectory
        if (collision_checker_.is_dubins_path_valid(traj)) {
            current_path_ = traj;
            plan_valid_ = true;
            ROS_INFO("[DubinsPlanner] Selected Candidate #%lu (Type: %d, Length: %.2f)", 
                     i + 1, (int)current_path_.type, current_path_.total_length);
            break;
        } else {
            // ROS_INFO("[DubinsPlanner] Candidate #%lu Collides...", i + 1);
        }
    }

    if (!plan_valid_) {
        ROS_WARN("[DubinsPlanner] All candidates collided!");
        current_path_ = candidates[0]; 
    }

    if (debug_viz) publishDebugViz(current_path_, plan_valid_);
    return plan_valid_;             // Return whether a valid plan was found
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
void DubinsPlanner::stop() {
    is_executing_ = false;
    // Send zero-velocity command
    robot_control::Reference ref_msg;
    ref_msg.v_d = 0.0;
    ref_msg.omega_d = 0.0;
    ref_msg.plan_finished = true;
    pub_ref_.publish(ref_msg);
}

// Real executor of the planned path
bool DubinsPlanner::spin(double curr_x, double curr_y, double curr_th) {
    if (!is_executing_) return false;

    // Access the current segment from the array
    const Kinematics::Segment& active_seg = current_path_.segments[current_segment_index_];
    double s_local = 0;

    if (std::abs(active_seg.curvature) < 1e-9) {
        // Straight line projection
        s_local = sqrt(pow(curr_x - active_seg.start_x, 2) + pow(curr_y - active_seg.start_y, 2));
    } else {
        // Arc logic using new segment members
        double R = 1.0 / active_seg.curvature;
        double center_x = active_seg.start_x - R * sin(active_seg.start_heading);
        double center_y = active_seg.start_y + R * cos(active_seg.start_heading);

        double angle_now = atan2(curr_y - center_y, curr_x - center_x);
        double angle_start = atan2(active_seg.start_y - center_y, active_seg.start_x - center_x);
        
        double delta_angle = angle_now - angle_start;
        if (active_seg.curvature > 0) { // Left
            while (delta_angle < 0) delta_angle += 2 * M_PI;
        } else { // Right
            while (delta_angle > 0) delta_angle -= 2 * M_PI;
        }
        s_local = std::abs(delta_angle * R);
    }

    // Segment switching with tolerance
    if (s_local >= (active_seg.length - 0.05)) {
        if (current_segment_index_ < 2) {
            current_segment_index_++;
            return false;
        } else {
            stop();
            return true; 
        }
    }

    publishReference(active_seg, s_local);
    return false;
}

void DubinsPlanner::publishReference(const Kinematics::Segment& seg, double s_local) {
    
    Kinematics::State ref_state;
    Kinematics::DubinsSolver::project_state(s_local, seg.start_x, seg.start_y, seg.start_heading, seg.curvature, ref_state);

    // Error Checking for NaNs
    if (std::isnan(ref_state.x) || std::isnan(ref_state.y) || std::isnan(ref_state.yaw)) {
        ROS_ERROR("[DubinsPlanner] NaN detected in reference projection. Stopping.");
        stop(); 
        return;
    }

    robot_control::Reference ref_msg;
    ref_msg.x_d = ref_state.x;
    ref_msg.y_d = ref_state.y;
    ref_msg.theta_d = ref_state.yaw;
    ref_msg.v_d = target_velocity_;
    ref_msg.omega_d = seg.curvature * target_velocity_; 
    ref_msg.plan_finished = false;

    pub_ref_.publish(ref_msg);
}

// Debug Visualization on RViz
void DubinsPlanner::publishDebugViz(const Kinematics::Trajectory& trajectory, bool valid) {
    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.header.frame_id = "map"; 
    m.header.stamp = ros::Time::now();
    m.ns = "dubins_plan";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = 0.05;
    m.pose.orientation.w = 1.0;
    
    if (valid) { m.color.g = 1.0; m.color.a = 1.0; } 
    else       { m.color.r = 1.0; m.color.a = 1.0; }

    auto add_segment_points = [&](const Kinematics::Segment& seg) {
        double step = 0.1;
        int n = std::ceil(seg.length / step);
        for(int i = 0; i <= n; i++) {
            double s = std::min(seg.length, i * step);
            Kinematics::State p_state;
            Kinematics::DubinsSolver::project_state(s, seg.start_x, seg.start_y, seg.start_heading, seg.curvature, p_state);
            
            geometry_msgs::Point p; 
            p.x = p_state.x; 
            p.y = p_state.y;
            m.points.push_back(p);
        }
    };

    for (const auto& seg : trajectory.segments) {
        add_segment_points(seg);
    }
    
    ma.markers.push_back(m);
    pub_viz_.publish(ma);
}