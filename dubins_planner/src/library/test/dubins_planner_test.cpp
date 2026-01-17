#include <ros/ros.h>
#include <robot_control/Reference.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include "dubins_planner/dubins_planner.h"
#include "map_library/map_builder.h"

class DubinsPlannerTester {
private:
    ros::NodeHandle nh_;
    DubinsPlanner planner_; 
    
    ros::Subscriber odom_sub_;
    double current_x_ = 0.0, current_y_ = 0.0, current_theta_ = 0.0;
    
    // Member variables to store goal
    Point goal_pos_;
    double goal_theta_ = 0.0;
    bool test_running_ = false;

public:
    DubinsPlannerTester() 
        : nh_("~"), 
          planner_(nh_, "limo0", 0.15, 0.05) 
    {
        odom_sub_ = nh_.subscribe("/limo0/odom", 1, &DubinsPlannerTester::odomCallback, this);
    }

    void setupTest() {
        // 1. Load Map
        map_builder::MapBuilder builder(nh_, 10.0);
        Map my_map = builder.buildMap();
        planner_.setMap(my_map);

        // 2. Determine Goal from Map or Defaults
        const auto& all_gates = my_map.gates.get_gates();
        const bool custom_goal = true;
        if (!all_gates.empty() && !custom_goal) {
            goal_pos_ = all_gates[0].get_position();
            const Orientation& o = all_gates[0].get_orientation();
            goal_theta_ = std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z));
        } else {
            goal_pos_.x = -5.0; goal_pos_.y = 2.0;
            goal_theta_ = M_PI / 2.0;
        }

        double min_turning_radius = 0.5;

        // 3. Plan using current position as start
        ROS_INFO("[Tester] Planning from current pose to goal...");
        if (planner_.planPath(current_x_, current_y_, current_theta_, 
                              goal_pos_.x, goal_pos_.y, goal_theta_, 
                              min_turning_radius, true)) {
            ROS_INFO("[Tester] SUCCESS.");
            planner_.startExecution(0.3);
            test_running_ = true;
        } else {
            ROS_ERROR("[Tester] FAILED.");
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double r, p;
        tf::Matrix3x3(q).getRPY(r, p, current_theta_);
    }

    void update() {
        if (test_running_) {
            bool finished = planner_.spin(current_x_, current_y_, current_theta_);
            if (finished) {
                ROS_INFO("[Tester] Goal Reached.");
                test_running_ = false;
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dubins_tester_node");
    DubinsPlannerTester tester;
    
    // Give ROS time to get initial odom before planning
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    
    tester.setupTest();

    ros::Rate loop_rate(20); 
    while (ros::ok()) {
        tester.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}