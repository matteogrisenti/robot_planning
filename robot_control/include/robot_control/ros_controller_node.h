// Implement a ROS node that wrap the LyapunovController

#ifndef ROS_CONTROLLER_NODE_H
#define ROS_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <robot_control/Reference.h> 
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>

#include <vector>
#include <string>
#include <memory>
#include <fstream>
#include <limits>

#include "lyapunov_controller.h"

// Define Vector6d type
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class RosControllerNode {
public:
    // Constructor and Destructor
    /**
     * @brief Constructor for RosControllerNode 
     * @param robot_name Name of the robot (e.g., "limo1")
     * @param debug If true, enables debug mode with additional logging
     */
    explicit RosControllerNode(const std::string& robot_name, bool debug);
    ~RosControllerNode();

    // Main control loop
    void startController();

private:
    // Initialization methods
    void initVars();
    void startPublisherSubscribers();
    
    // Callbacks
    void receivePose(const nav_msgs::Odometry::ConstPtr& msg);
    void receiveReference(const robot_control::Reference::ConstPtr& msg);
    
    // Helpers
    void sendCommands(double lin_vel, double ang_vel);
    void logData();
    void plotData();
    Eigen::Vector3d unwrapVector(const Eigen::Vector3d& vec, const Eigen::Vector3d& old_vec);

    // Members
    std::string robot_name_;    
    bool debug_;
    ros::NodeHandle nh_;
    ros::Publisher command_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber ref_sub_;

    // Controller
    std::unique_ptr<LyapunovController> controller_;
    RobotState robot_state_;

    // State Variables
    Vector6d base_pose_w_;
    Vector6d base_twist_w_;
    Eigen::Quaterniond quaternion_;
    Eigen::Vector3d euler_old_;
    
    double ctrl_v_;         // Control linear velocity
    double ctrl_omega_;     // Control angular velocity
    double v_d_;            // Desired linear velocity
    double omega_d_;        // Desired angular velocity
    
    double old_theta_;
    double time_;
    double dt_;
    int log_counter_;

    // Desired state
    double des_x_;
    double des_y_;
    double des_theta_;

    // Logging Data Structures
    Eigen::MatrixXd base_pose_w_log_;
    Eigen::MatrixXd base_twist_w_log_;
    Eigen::VectorXd time_log_;
    Eigen::MatrixXd state_log_;
    Eigen::MatrixXd des_state_log_;
};

#endif // ROS_CONTROLLER_NODE_H