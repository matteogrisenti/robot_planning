#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>

#include "dubins_planner/dubins_planner.h"
#include "map/map_builder.h"
#include "robot_control/Reference.h"

// --- LEGACY GLOBAL VARIABLES ---
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
    ros::Subscriber sub_ref_;

    std::string robot_name_;

    DubinsPlanner planner_;

    bool odom_received_ = false;
    bool plan_started_ = false;
    bool plan_finished_ = false; 

    double robot_x_ = 0.0;     
    double robot_y_ = 0.0;     
    double robot_theta_ = 0.0; 

    Point goal_pos_;
    double goal_theta_;
    double target_speed_ = 0.5; 
    double min_turning_radius_ = 0.5; 

    double ref_x_ = 0.0, ref_y_ = 0.0, ref_theta_ = 0.0;
    bool ref_received_ = false;

    double sum_sq_error_pos_ = 0.0;  
    double sum_sq_error_theta_ = 0.0; 
    long sample_count_ = 0;           

public:
    ControlTestNode(const std::string& robot_name)
        : robot_name_(robot_name), planner_(nh_, robot_name_, 0.25, 0.05) 
    {
        sub_odom_ = nh_.subscribe("/" + robot_name_ + "/odom", 1, &ControlTestNode::odomCallback, this);
        sub_ref_ = nh_.subscribe("/" + robot_name_ + "/ref", 1, &ControlTestNode::refCallback, this);

        ROS_INFO("[ControlTestNode] Started for %s. Waiting for Odom...", robot_name_.c_str());
        loadMapAndConfigurePlanner();
    }

    void loadMapAndConfigurePlanner() {
        ROS_INFO("[ControlTestNode] Loading Map...");
        
        map_builder::MapBuilder builder(nh_, 100.0);
        Map map = builder.buildMap();
        planner_.setMap(map);

        const auto& all_gates = map.gates.get_gates();
        if (!all_gates.empty()) {
            goal_pos_ = all_gates[0].get_position();
            const Orientation& o = all_gates[0].get_orientation();
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

    void refCallback(const robot_control::Reference::ConstPtr& msg) {
        if (msg->plan_finished) {
            plan_finished_ = true;
            reportAccuracy();
        } else {
            ref_x_ = msg->x_d;
            ref_y_ = msg->y_d;
            ref_theta_ = msg->theta_d;
            ref_received_ = true;
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double roll, pitch, robot_theta;
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, robot_theta);
        robot_theta_ = robot_theta;

        if (!odom_received_) {
            ROS_INFO("[ControlTestNode] Initial Odom Received: [%.2f, %.2f, %.2f]", robot_x_, robot_y_, robot_theta_);
            odom_received_ = true;
            triggerPlanning(robot_x_, robot_y_, robot_theta_);
        }

        if (plan_started_ && !plan_finished_ && ref_received_) {
            computeInstantaneousError();
        }
    }

    void triggerPlanning(double start_x, double start_y, double start_th) {
        ROS_INFO("[ControlTestNode] Requesting Plan...");
        bool success = planner_.planPath(start_x, start_y, start_th, 
                                         goal_pos_.x, goal_pos_.y, goal_theta_, 
                                         min_turning_radius_);
        if (success) {
            ROS_INFO("[ControlTestNode] Plan Successful! Starting Execution...");
            planner_.startExecution(target_speed_);
            plan_started_ = true;
        } else {
            ROS_ERROR("[ControlTestNode] Planning Failed! Retrying on next odom update...");
        }
    }

    void computeInstantaneousError() {
        double err_x = ref_x_ - robot_x_;
        double err_y = ref_y_ - robot_y_;
        double sq_err_pos = err_x * err_x + err_y * err_y;

        double err_th = ref_theta_ - robot_theta_;
        err_th = std::atan2(std::sin(err_th), std::cos(err_th)); // normalized

        double sq_err_th = err_th * err_th;

        sum_sq_error_pos_ += sq_err_pos;
        sum_sq_error_theta_ += sq_err_th;
        sample_count_++;
    }

    void reportAccuracy() {
        if (sample_count_ == 0) {
            ROS_WARN("[ControlTestNode] No samples collected for accuracy.");
            return;
        }

        double rmse_pos = std::sqrt(sum_sq_error_pos_ / sample_count_);
        double rmse_th  = std::sqrt(sum_sq_error_theta_ / sample_count_);

        ROS_INFO("========================================");
        ROS_INFO("       TRAJECTORY TRACKING ACCURACY     ");
        ROS_INFO("========================================");
        ROS_INFO(" Samples collected: %ld", sample_count_);
        ROS_INFO(" Position RMSE:     %.4f meters", rmse_pos);
        ROS_INFO(" Heading RMSE:      %.4f radians", rmse_th);
        ROS_INFO("========================================");

        sample_count_ = 0;
        sum_sq_error_pos_ = 0.0;
        sum_sq_error_theta_ = 0.0;
    }

    void update() {
        if (plan_started_) {
            planner_.spin();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_test_node");
    ros::NodeHandle nh_("~");

    std::string robot_name = nh_.param<std::string>("robot_name", "limo0");
    ControlTestNode node(robot_name);

    ros::Rate r(50); 
    while (ros::ok()) {
        ros::spinOnce();
        node.update();
        r.sleep();
    }
    return 0;
}
