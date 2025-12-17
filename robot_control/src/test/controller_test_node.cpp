/**
 * @file controller_test_node.cpp
 * @brief Orchestrator node that uses DubinsPlanner to drive the robot to a gate.
 * * RESPONSIBILITIES:
 * 1. Load the Map and find the Goal (Gate).
 * 2. Wait for the Robot's initial Odometry.
 * 3. Invoke DubinsPlanner to plan and execute the trajectory.
 * * TESTING:
 * 4. Test the controller logic in a simulated environment.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// Include the Planner
#include "dubins_planner/dubins_planner.h"
#include "map/map_builder.h"

// --- LEGACY GLOBAL VARIABLES (Required for linking dubins_trajectory library) ---
// These define the state for the C-style library functions
bool DEBUG = false;
long double X0, Y0, Th0, Xf, Yf, Thf, Kmax;
int pidx;
int no_waypts, step, no_of_samples;
long double angle_step;
dubinscurve_out dubin_curve;
point init_pt, final_pt;
std::vector<point> best_path;
// --------------------------------------------------------------------------------

class ControlTestNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_;

    // The Planner Object (Handles logic, collision, and publishing)
    DubinsPlanner planner_;

    std::string robot_name_; 

    // State Variables
    bool odom_received_ = false;
    bool plan_started_ = false;

    // Goal Data
    Point goal_pos_;
    double goal_theta_;
    double target_speed_ = 0.5; // m/s
    double min_turning_radius_ = 0.5; // meters

public:
    ControlTestNode() 
        : nh_("~"),
          // Initialize Planner with robot name, radius (0.25), and safety margin (0.05)
          planner_(nh_, "limo0", 0.25, 0.05) 
    {
        // 1. Get Parameters
        nh_.param<std::string>("robot_name", robot_name_, "limo0");
        
        // 2. Setup Communication
        // We only listen to Odom to know WHERE TO START.
        // The Planner handles publishing references to /limo0/ref internally.
        sub_odom_ = nh_.subscribe("/" + robot_name_ + "/odom", 1, &ControlTestNode::odomCallback, this);

        ROS_INFO("[ControlTestNode] Started for %s. Waiting for Odom...", robot_name_.c_str());

        // 3. Load Map and Set Goal
        loadMapAndConfigurePlanner();
    }

    void loadMapAndConfigurePlanner() {
        ROS_INFO("[ControlTestNode] Loading Map...");
        
        // Build Map
        map_builder::MapBuilder builder(nh_, 100.0);
        Map map = builder.buildMap();
        
        // Pass map to planner for collision checking
        planner_.setMap(map);

        // Find Goal (First Gate)
        const auto& all_gates = map.gates.get_gates();
        if (!all_gates.empty()) {
            goal_pos_ = all_gates[0].get_position();
            
            // Calculate orientation: perpendicular to the gate
            const Orientation& o = all_gates[0].get_orientation();
            // Convert Quat to Yaw
            double roll, pitch;
            tf::Quaternion q(o.x, o.y, o.z, o.w);
            tf::Matrix3x3(q).getRPY(roll, pitch, goal_theta_);
            
            ROS_INFO("[ControlTestNode] Goal set to Gate: [x: %.2f, y: %.2f, th: %.2f]", 
                     goal_pos_.x, goal_pos_.y, goal_theta_);
        } else {
            ROS_WARN("[ControlTestNode] No gates found! Defaulting to (2.0, 0.0)");
            goal_pos_ = {2.0, 0.0, 0.0};
            goal_theta_ = 0.0;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // We only need the initial pose to plan the path once.
        if (plan_started_) return;

        double robot_x = msg->pose.pose.position.x;
        double robot_y = msg->pose.pose.position.y;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, robot_theta;
        m.getRPY(roll, pitch, robot_theta);

        if (!odom_received_) {
            ROS_INFO("[ControlTestNode] Initial Odom Received: [%.2f, %.2f, %.2f]", 
                     robot_x, robot_y, robot_theta);
            odom_received_ = true;
            
            triggerPlanning(robot_x, robot_y, robot_theta);
        }
    }

    void triggerPlanning(double start_x, double start_y, double start_th) {
        ROS_INFO("[ControlTestNode] Requesting Plan...");
        
        // 1. Plan the Path
        // This calculates geometry AND checks collisions
        bool success = planner_.planPath(start_x, start_y, start_th, 
                                         goal_pos_.x, goal_pos_.y, goal_theta_, 
                                         min_turning_radius_);

        if (success) {
            ROS_INFO("[ControlTestNode] Plan Successful! Starting Execution...");
            // 2. Start Execution (Planner will begin publishing refs)
            planner_.startExecution(target_speed_);
            plan_started_ = true;
        } else {
            ROS_ERROR("[ControlTestNode] Planning Failed! (Collision or Geometric issue). Retrying on next odom update...");
            // plan_started_ remains false, so we will try again next callback
        }
    }

    // Main loop function called from main()
    void update() {
        // The planner needs to be spun to publish the time-varying reference
        if (plan_started_) {
            planner_.spin();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_test_node");
    
    ControlTestNode node;

    // Run at high frequency (50Hz) to ensure smooth reference publishing
    ros::Rate r(50); 
    while (ros::ok()) {
        ros::spinOnce();
        node.update(); // Drives the planner execution
        r.sleep();
    }

    return 0;
}