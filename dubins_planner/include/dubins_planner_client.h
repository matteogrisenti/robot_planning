#ifndef DUBINS_CLIENT_H
#define DUBINS_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <string>

#include <dubins_planner/FollowDubinsAction.h> 

class DubinsClient {
private:
    // Define the Action Client type
    typedef actionlib::SimpleActionClient<dubins_planner::FollowDubinsAction> Client;
    
    Client ac_;
    std::string action_name_;

public:
    /**
     * @param name The name of the action server 
     */
    DubinsClient(std::string name);

    /**
     * Sends a goal to the Dubins Server
     * @param x Target X
     * @param y Target Y
     * @param theta Target Orientation (radians)
     * @param v Linear Velocity
     * @param r Turning Radius
     */
    void sendGoal(double x, double y, double theta, double v, double r);

    /**
     * Wait for the current goal to finish
     * @param timeout_sec Timeout in seconds (default 60.0)
     */
    bool waitForResult(double timeout_sec = 60.0);

    /**
     * Check if the last goal succeeded
     */
    bool isSuccess();

    // --- Callbacks ---
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const dubins_planner::FollowDubinsResultConstPtr& result);

    void activeCb();

    void feedbackCb(const dubins_planner::FollowDubinsFeedbackConstPtr& feedback);
};

#endif // DUBINS_CLIENT_H