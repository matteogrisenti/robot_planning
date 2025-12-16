#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h> 

// Include your custom message for the controller
#include <loco_planning/Reference.h>

// Include your map and dubins libraries
#include "map/map_builder.h"
#include "dubins_planner/dubins_trajectory.h"

// --- Global Variables Required by dubins_trajectory.cpp ---
// These are necessary because the provided dubins library relies on them.
// We define them here if they are externs, or rely on the linked cpp.
// Based on your file, they are defined in dubins_trajectory.cpp, so we just declare usage.
extern bool DEBUG;
extern int no_of_samples;

class DubinsReferenceNode {
private:
    ros::NodeHandle nh;
    ros::Publisher pub_ref;
    ros::Subscriber sub_odom;

    std::string robot_name;

    // State Variables
    double robot_x, robot_y, robot_theta;
    bool odom_received = false;
    bool path_calculated = false;
    bool goal_reached = false;

    // Dubins Path Data
    dubinscurve_out curve;
    double total_length;
    
    // Timing and Execution
    ros::Time start_time;
    double TARGET_SPEED = 0.5; // m/s
    
    // Goal Data
    Point goal_pos;
    double goal_theta;

public:
    DubinsReferenceNode() {
        // 1. Get Parameters
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("robot_name", robot_name, "limo0");
        
        // 2. Setup Communication
        // Publishes to the topic controller.py listens to: /{robot_name}/ref
        pub_ref = nh.advertise<loco_planning::Reference>("/" + robot_name + "/ref", 1);
        sub_odom = nh.subscribe("/" + robot_name + "/odom", 1, &DubinsReferenceNode::odomCallback, this);

        ROS_INFO("DubinsReferenceNode started for %s. Waiting for Odom...", robot_name.c_str());

        // 3. Load Map and Set Goal
        loadMap();
    }

    void loadMap() {
        ROS_INFO("Loading Map to find Goal...");
        map_builder::MapBuilder builder(nh, 100.0);
        Map map_ = builder.buildMap();
        const auto& all_gates = map_.gates.get_gates();

        if (!all_gates.empty()) {
            goal_pos = all_gates[0].get_position();
            
            // Calculate orientation: perpendicular to the gate
            const Orientation& o = all_gates[0].get_orientation();
            // Convert Quat to Yaw
            double roll, pitch;
            tf::Quaternion q(o.x, o.y, o.z, o.w);
            tf::Matrix3x3(q).getRPY(roll, pitch, goal_theta);
            
            ROS_INFO("Goal set to Gate: [x: %.2f, y: %.2f, th: %.2f]", goal_pos.x, goal_pos.y, goal_theta);
        } else {
            ROS_WARN("No gates found! Defaulting to (2.0, 0.0)");
            goal_pos = {2.0, 0.0, 0.0};
            goal_theta = 0.0;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // We only really need odom ONCE to calculate the start path.
        // After that, we just execute the plan blindly (Open Loop Reference).
        // The controller handles the closed-loop error correction.
        
        if (path_calculated) return; // Stop updating start pose once we are moving

        robot_x = msg->pose.pose.position.x;
        robot_y = msg->pose.pose.position.y;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, robot_theta);

        if (!odom_received) {
            ROS_INFO("Initial Odom Received: [%.2f, %.2f, %.2f]", robot_x, robot_y, robot_theta);
            odom_received = true;
            calculatePath();
        }
    }

    void calculatePath() {
        if (!odom_received) return;

        double rho = 0.5; // Min turning radius (adjust based on Limo capabilities)
        int best_idx = -1;

        ROS_INFO("Calculating Dubins Path...");
        dubins_shortest_path(robot_x, robot_y, robot_theta, 
                             goal_pos.x, goal_pos.y, goal_theta, 
                             1.0/rho, best_idx, &curve);

        if (best_idx >= 0) {
            path_calculated = true;
            total_length = curve.L;
            start_time = ros::Time::now();
            ROS_INFO("Path Calculated! Length: %.2f meters. Starting Execution.", total_length);
        } else {
            ROS_ERROR("Failed to find a valid Dubins path!");
        }
    }

    void updateLoop() {
        if (!path_calculated) return;
        if (goal_reached) return;

        // 1. Calculate time elapsed
        ros::Duration dt = ros::Time::now() - start_time;
        double t_sec = dt.toSec();

        // 2. Determine current distance on path
        double current_dist = t_sec * TARGET_SPEED;

        // 3. Define Segment Lengths
        double l1 = curve.a1.l;
        double l2 = curve.a2.l;
        double l3 = curve.a3.l;

        // 4. Check if finished
        if (current_dist >= (l1 + l2 + l3)) {
            ROS_INFO("Trajectory Finished.");
            publishReference(curve.a3.xf, curve.a3.yf, curve.a3.thf, 0.0, 0.0, true);
            goal_reached = true;
            return;
        }

        // 5. Find which segment we are on and calculate State
        dubinsarc_out* target_arc = nullptr;
        double s_segment = 0.0; // Distance within the current segment

        if (current_dist < l1) {
            // Segment 1
            target_arc = &curve.a1;
            s_segment = current_dist;
        } else if (current_dist < (l1 + l2)) {
            // Segment 2
            target_arc = &curve.a2;
            s_segment = current_dist - l1;
        } else {
            // Segment 3
            target_arc = &curve.a3;
            s_segment = current_dist - (l1 + l2);
        }

        // 6. Calculate desired point (x,y,theta) using library function
        long double ref_x, ref_y, ref_th;
        circline(s_segment, target_arc->x0, target_arc->y0, target_arc->th0, target_arc->k, ref_x, ref_y, ref_th);

        // 7. Calculate desired velocities (Feedforward)
        double ref_v = TARGET_SPEED;
        double ref_omega = target_arc->k * TARGET_SPEED; // omega = k * v

        // 8. Publish
        publishReference((double)ref_x, (double)ref_y, (double)ref_th, ref_v, ref_omega, false);
    }

    void publishReference(double x, double y, double th, double v, double w, bool finished) {
        loco_planning::Reference ref_msg;
        ref_msg.x_d = x;
        ref_msg.y_d = y;
        ref_msg.theta_d = th;
        ref_msg.v_d = v;
        ref_msg.omega_d = w;
        ref_msg.plan_finished = finished;
        
        pub_ref.publish(ref_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dubins_reference_node");
    
    DubinsReferenceNode node;

    // Run at high frequency to give smooth references to the controller
    ros::Rate r(50); // 50 Hz
    while (ros::ok()) {
        ros::spinOnce();
        node.updateLoop();
        r.sleep();
    }

    return 0;
}