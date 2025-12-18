#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>
#include <iostream>

// Action Library
#include <actionlib/server/simple_action_server.h>
#include "dubins_planner/FollowDubinsAction.h" 

#include "dubins_planner/dubins_planner.h"
#include "map_library/map_builder.h"

class DubinsActionServer {
protected:
    ros::NodeHandle nh_;
    // Action Server Object
    actionlib::SimpleActionServer<dubins_planner::FollowDubinsAction> as_; 
    
    std::string action_name_;
    dubins_planner::FollowDubinsFeedback feedback_;
    dubins_planner::FollowDubinsResult result_;

    // Logic
    std::shared_ptr<DubinsPlanner> planner_logic_;
    ros::Subscriber odom_sub_;
    Map map_;
    std::string robot_name_;

    // State
    double current_x_, current_y_, current_theta_;
    bool has_odom_;

public:
    DubinsActionServer(std::string name) : 
        nh_("~"),
        as_(nh_, name, boost::bind(&DubinsActionServer::executeCB, this, _1), false),
        action_name_(" DUBINS PLANNER SERVER ")
    {
        // 1. Parametri
        nh_.param<std::string>("robot_name", robot_name_, "limo0");

        // 2. Setup Logic
        planner_logic_ = std::make_shared<DubinsPlanner>(nh_, robot_name_, 0.25, 0.05);

        // 3. Map
        ROS_INFO("[%s] Building Map...", action_name_.c_str());
        map_builder::MapBuilder builder(nh_, 100.0);
        map_ = builder.buildMap();
        planner_logic_->setMap(map_);

        // 4. Odom
        odom_sub_ = nh_.subscribe("/" + robot_name_ + "/odom", 1, &DubinsActionServer::odomCallback, this);
        has_odom_ = false;

        // 5. Start Action Server
        as_.start();
        ROS_INFO("[%s] Action Server Started.", action_name_.c_str());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        tf::Quaternion q(
            msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y, 
            msg->pose.pose.orientation.z, 
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        current_theta_ = y;
        has_odom_ = true;
    }

    // --- EXECUTION CALLBACK ---
    void executeCB(const dubins_planner::FollowDubinsGoalConstPtr &goal) {
        ros::Rate r(50);
        bool success = false; // Default false

        ROS_INFO("[%s] New Goal: (%.2f, %.2f)", action_name_.c_str(), goal->goal_x, goal->goal_y);

        if (!has_odom_) {
            result_.success = false;
            result_.message = "No Odometry";
            as_.setAborted(result_);
            return;
        }

        // --- PHASE 1: PLANNING ---
        bool plan_ok = planner_logic_->planPath(
            current_x_, current_y_, current_theta_,
            goal->goal_x, goal->goal_y, goal->goal_theta,
            goal->turning_radius, true
        );

        // Check if planning was successful
        if (!plan_ok) {
            result_.success = false;
            result_.message = "Planning Collision/Failed";
            as_.setAborted(result_);
            return;
        }

        // --- PHASE 2: EXECUTION ---
        planner_logic_->startExecution(goal->velocity);
        
        // Main Execution Loop
        while(ros::ok()) {
            // 1. Check Cancellation (Preempt)
            if (as_.isPreemptRequested()) {
                ROS_WARN("[%s] Preempted.", action_name_.c_str());
                planner_logic_->stop();
                as_.setPreempted();
                success = false;
                break;
            }

            // 2. Step the Planner Logic
            bool path_completed = planner_logic_->spin();

            // 3. Feedback: 
            //      - Distance to Goal
            //      - Current Position
            //      - Status (Arrived/Moving)
            double dist_sq = pow(current_x_ - goal->goal_x, 2) + pow(current_y_ - goal->goal_y, 2);
            feedback_.distance_to_goal = sqrt(dist_sq);
            feedback_.current_x = current_x_;
            feedback_.current_y = current_y_;
            feedback_.status = path_completed ? "Arrived" : "Moving";
            as_.publishFeedback(feedback_);

            // 4. Check Arrival 
            if (path_completed) {
                ROS_INFO("[%s] Planner reported completion.", action_name_.c_str());
                success = true;
                break; // Usciamo dal while
            }

            r.sleep();
        }

        if(success) {
            result_.success = true;
            result_.message = "Arrived";
            as_.setSucceeded(result_);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dubins_action_server");
    DubinsActionServer server("follow_dubins_path");
    
    // AsyncSpinner essenziale per leggere Odom mentre siamo nel loop
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    ros::waitForShutdown();
    return 0;
}