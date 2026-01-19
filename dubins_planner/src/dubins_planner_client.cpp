#include "dubins_planner_client.h"
#include <ros/ros.h>

DubinsClient::DubinsClient(std::string name) : ac_(name, true), action_name_(name) {
    ROS_INFO("[Dubins Client] Waiting for action server [%s] to start...", action_name_.c_str());
    ac_.waitForServer(); 
    ROS_INFO("[Dubins Client] Action server [%s] started, sending goals.", action_name_.c_str());
}

void DubinsClient::sendGoal(double x, double y, double theta, double v, double r) {
    dubins_planner::FollowDubinsGoal goal;
    goal.goal_x = x;
    goal.goal_y = y;
    goal.goal_theta = theta;
    goal.velocity = v;
    goal.turning_radius = r;

    ROS_INFO("[Dubins Client] Sending goal: (%.2f, %.2f) @ %.2f rad", x, y, theta);
    
    // Register callbacks
    ac_.sendGoal(goal,
            boost::bind(&DubinsClient::doneCb, this, _1, _2),
            boost::bind(&DubinsClient::activeCb, this),
            boost::bind(&DubinsClient::feedbackCb, this, _1));
}

bool DubinsClient::waitForResult(double timeout_sec) {
    // Note: The default value (60.0) is handled in the header file
    return ac_.waitForResult(ros::Duration(timeout_sec));
}

bool DubinsClient::isSuccess() {
    return (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
}

// --- Callbacks ---

void DubinsClient::doneCb(const actionlib::SimpleClientGoalState& state,
            const dubins_planner::FollowDubinsResultConstPtr& result) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("[Dubins Client] Goal Reached: %s", result->message.c_str());
    } else {
        ROS_WARN("[Dubins Client] Goal Failed/Preempted: %s", result->message.c_str());
    }
}

void DubinsClient::activeCb() {
    // ROS_INFO("Goal just went active");
}

void DubinsClient::feedbackCb(const dubins_planner::FollowDubinsFeedbackConstPtr& feedback) {
    // Verbose feedback log
    //ROS_INFO("[Dubins Client] Dist to goal: %.2f | Status: %s", feedback->distance_to_goal, feedback->status.c_str());
}