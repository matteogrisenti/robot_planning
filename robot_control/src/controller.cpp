/**
 * @file controller.cpp
 * @brief ROS C++ implementation of unicycle controller with Lyapunov-based control
 * @author Converted from Python version by mfocchi
 * @date 2018-11-02 (original), converted 2025
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <robot_control/Reference.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <memory>
#include <fstream>

// You'll need to create these headers based on your Python modules:
// #include "utils/velocity_generator.h"
// #include "utils/trajectory_generator.h"
// #include "utils/math_tools.h"
// #include "controllers/lyapunov.h"
// #include "params.h"

using namespace Eigen;

// Define Vector6d type BEFORE using it
typedef Matrix<double, 6, 1> Vector6d;

/**
 * @brief Unwraps an angle to ensure continuity
 */
inline double unwrapAngle(double angle, double old_angle) {
    double diff = angle - old_angle;
    if (diff > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (diff < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

/**
 * @brief Unwraps a vector of angles
 */
inline Vector3d unwrapVector(const Vector3d& vec, const Vector3d& old_vec) {
    Vector3d result;
    for (int i = 0; i < 3; ++i) {
        result(i) = unwrapAngle(vec(i), old_vec(i));
    }
    return result;
}

/**
 * @brief Robot state structure
 */
struct RobotState {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

/**
 * @brief Lyapunov controller parameters
 */
struct LyapunovParams {
    double K_P;
    double K_THETA;
    double DT;
    
    LyapunovParams(double kp, double kth, double dt) 
        : K_P(kp), K_THETA(kth), DT(dt) {}
};

/**
 * @brief Simple Lyapunov controller implementation
 */
class LyapunovController {
public:
    explicit LyapunovController(const LyapunovParams& params) 
        : params_(params) {}
    
    std::pair<double, double> controlUnicycle(
        const RobotState& robot_state,
        double time,
        double des_x, double des_y, double des_theta,
        double v_d, double omega_d,
        bool verbose = false)
    {
        // Position error
        double e_x = des_x - robot_state.x;
        double e_y = des_y - robot_state.y;
        
        // Heading error
        double e_theta = des_theta - robot_state.theta;
        e_theta = atan2(sin(e_theta), cos(e_theta)); // Normalize to [-pi, pi]
        
        // Control law
        double rho = sqrt(e_x * e_x + e_y * e_y);
        double alpha = atan2(e_y, e_x) - robot_state.theta;
        alpha = atan2(sin(alpha), cos(alpha));
        
        double ctrl_v = v_d + params_.K_P * rho * cos(alpha);
        double ctrl_omega = omega_d + params_.K_THETA * e_theta;
        
        if (verbose) {
            ROS_INFO("e_x: %.3f, e_y: %.3f, e_theta: %.3f", e_x, e_y, e_theta);
            ROS_INFO("ctrl_v: %.3f, ctrl_omega: %.3f", ctrl_v, ctrl_omega);
        }
        
        return {ctrl_v, ctrl_omega};
    }
    
private:
    LyapunovParams params_;
};

/**
 * @brief Main controller class
 */
class Controller {
public:
    explicit Controller(const std::string& robot_name = "limo1", bool debug = false)
        : robot_name_(robot_name), debug_(debug), nh_("~") 
    {
        ROS_INFO("[Controller] Controller initialized for robot: %s", robot_name_.c_str());
    }
    
    ~Controller() {
        sendCommands(0.0, 0.0);
        if (debug_) {
            plotData();
        }
    }
    
    void startController() {
        // Initialize ROS node is done in main
        ros::NodeHandle nh_robot("/" + robot_name_);
        
        initVars();
        startPublisherSubscribers();
        
        // Get parameters (you'll need to load these from your config)
        double k_p = 1.0;    // Load from params
        double k_th = 2.0;   // Load from params
        double dt = 0.01;    // Load from params
        
        nh_.param("k_p", k_p, 1.0);
        nh_.param("k_th", k_th, 2.0);
        nh_.param("dt", dt, 0.01);
        
        LyapunovParams params(k_p, k_th, dt);
        controller_ = std::make_unique<LyapunovController>(params);
        
        dt_ = dt;
        ros::Rate rate(1.0 / dt);
        
        ROS_INFO("[Controller] Starting control loop at %.1f Hz", 1.0/dt);
        
        while (ros::ok()) {
            try {
                // Update robot state
                robot_state_.x = base_pose_w_(0);
                robot_state_.y = base_pose_w_(1);
                robot_state_.theta = base_pose_w_(5);
                
                if (debug_) {
                    // Update trajectory (if using trajectory generator)
                    // You'll need to implement this based on your trajectory class
                    // bool traj_finished = updateTrajectory();
                    // if (traj_finished) {
                    //     ROS_INFO("Ending simulation...");
                    //     plotData();
                    //     sendCommands(0.0, 0.0);
                    //     ros::shutdown();
                    //     return;
                    // }
                }
                
                // Unwrap desired angle
                des_theta_ = unwrapAngle(des_theta_, old_theta_);
                old_theta_ = des_theta_;
                
                // Compute control
                auto [ctrl_v, ctrl_omega] = controller_->controlUnicycle(
                    robot_state_, time_,
                    des_x_, des_y_, des_theta_,
                    v_d_, omega_d_,
                    false
                );
                
                ctrl_v_ = ctrl_v;
                ctrl_omega_ = ctrl_omega;
                
                sendCommands(ctrl_v_, ctrl_omega_);
                logData();
                
                rate.sleep();
                time_ += dt_;
                time_ = std::round(time_ * 10000.0) / 10000.0; // Round to 4 decimals
                
                ros::spinOnce();
                
            } catch (const ros::Exception& e) {
                ROS_ERROR("[Controller] ROS Exception: %s", e.what());
                sendCommands(0.0, 0.0);
                plotData();
                break;
            }
        }
    }
    
private:
    void initVars() {
        base_pose_w_.setZero();
        base_twist_w_.setZero();
        
        ctrl_v_ = 0.0;
        ctrl_omega_ = 0.0;
        v_d_ = 0.1;
        omega_d_ = 0.0;
        
        quaternion_ = Quaterniond(1.0, 0.0, 0.0, 0.0); // w, x, y, z
        euler_old_.setZero();
        old_theta_ = 0.0;
        time_ = 0.0;
        log_counter_ = 0;
        
        // Initialize log vectors
        int buffer_size = 10000; // Load from params
        nh_.param("buffer_size", buffer_size, 10000);
        
        base_pose_w_log_.resize(6, buffer_size);
        base_twist_w_log_.resize(6, buffer_size);
        time_log_.resize(buffer_size);
        state_log_.resize(3, buffer_size);
        des_state_log_.resize(3, buffer_size);
        
        base_pose_w_log_.fill(std::numeric_limits<double>::quiet_NaN());
        base_twist_w_log_.fill(std::numeric_limits<double>::quiet_NaN());
        time_log_.fill(std::numeric_limits<double>::quiet_NaN());
        state_log_.fill(std::numeric_limits<double>::quiet_NaN());
        des_state_log_.fill(std::numeric_limits<double>::quiet_NaN());
        
        // Initialize desired state
        // You may want to get initial state from odometry here
        des_x_ = 0.0;
        des_y_ = 0.0;
        des_theta_ = 0.0;
        
        ROS_INFO("[Controller] Variables initialized");
    }
    
    void startPublisherSubscribers() {
        ros::NodeHandle nh_robot("/" + robot_name_);
        
        command_pub_ = nh_robot.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        
        odom_sub_ = nh_robot.subscribe("odom", 1, 
            &Controller::receivePose, this,
            ros::TransportHints().tcpNoDelay());
            
        ref_sub_ = nh_robot.subscribe("ref", 1,
            &Controller::receiveReference, this,
            ros::TransportHints().tcpNoDelay());
        
        ROS_INFO("[Controller] Publishers and subscribers started");
    }
    
    void receivePose(const nav_msgs::Odometry::ConstPtr& msg) {
        // Extract quaternion
        quaternion_ = Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
        
        // Convert to euler angles
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        Vector3d euler(roll, pitch, yaw);
        euler = unwrapVector(euler, euler_old_);
        euler_old_ = euler;
        
        // Update pose
        base_pose_w_(0) = msg->pose.pose.position.x;
        base_pose_w_(1) = msg->pose.pose.position.y;
        base_pose_w_(2) = msg->pose.pose.position.z;
        base_pose_w_(3) = euler(0);
        base_pose_w_(4) = euler(1);
        base_pose_w_(5) = euler(2);
        
        // Update twist
        base_twist_w_(0) = msg->twist.twist.linear.x;
        base_twist_w_(1) = msg->twist.twist.linear.y;
        base_twist_w_(2) = msg->twist.twist.linear.z;
        base_twist_w_(3) = msg->twist.twist.angular.x;
        base_twist_w_(4) = msg->twist.twist.angular.y;
        base_twist_w_(5) = msg->twist.twist.angular.z;
    }
    
    void receiveReference(const robot_control::Reference::ConstPtr& msg) {
        if (msg->plan_finished) {
            ROS_INFO("[Controller] Plan finished, plotting data...");
            plotData();
        } else {
            des_x_ = msg->x_d;
            des_y_ = msg->y_d;
            des_theta_ = msg->theta_d;
            v_d_ = msg->v_d;
            omega_d_ = msg->omega_d;
            
            /* Debugging 
            ROS_INFO_STREAM("\033[31mReceived " << robot_name_ 
                << " des_x: " << des_x_ 
                << ", des_y: " << des_y_ 
                << ", des_theta: " << des_theta_
                << ", des_v: " << v_d_
                << ", des_omega: " << omega_d_ << "\033[0m");
            */
        }
    }
    
    void sendCommands(double lin_vel, double ang_vel) {
        geometry_msgs::Twist msg;
        msg.linear.x = lin_vel;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = ang_vel;
        
        command_pub_.publish(msg);
    }
    
    void logData() {
        if (log_counter_ < time_log_.size()) {
            des_state_log_(0, log_counter_) = des_x_;
            des_state_log_(1, log_counter_) = des_y_;
            des_state_log_(2, log_counter_) = des_theta_;
            
            state_log_(0, log_counter_) = base_pose_w_(0);
            state_log_(1, log_counter_) = base_pose_w_(1);
            state_log_(2, log_counter_) = base_pose_w_(5);
            
            base_pose_w_log_.col(log_counter_) = base_pose_w_;
            base_twist_w_log_.col(log_counter_) = base_twist_w_;
            time_log_(log_counter_) = time_;
            
            log_counter_++;
        }
    }
    
    void plotData() {
        // In C++, you would typically save data to file for plotting
        // or use a plotting library like matplotlib-cpp
        
        // Save to home directory for easy access
        std::string home = getenv("HOME");
        std::string filename = home + "/ros_ws/src/robot_control/src/test/" + robot_name_ + "_trajectory_data.txt";
        std::ofstream file(filename);
        
        if (file.is_open()) {
            file << "# time x y theta des_x des_y des_theta\n";
            for (int i = 0; i < log_counter_; ++i) {
                file << time_log_(i) << " "
                     << state_log_(0, i) << " "
                     << state_log_(1, i) << " "
                     << state_log_(2, i) << " "
                     << des_state_log_(0, i) << " "
                     << des_state_log_(1, i) << " "
                     << des_state_log_(2, i) << "\n";
            }
            file.close();
            ROS_INFO("[Controller] Data saved to %s", filename.c_str());
        } else {
            ROS_ERROR("[Controller] Could not open file for writing: %s", filename.c_str());
        }
        
        // You could also implement plotting using matplotlib-cpp or gnuplot-iostream
        ROS_INFO("[Controller] Plotting complete. Data logged for %d samples", log_counter_);
    }
    
    // Member variables
    std::string robot_name_;
    bool debug_;
    ros::NodeHandle nh_;
    
    ros::Publisher command_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber ref_sub_;
    
    Vector6d base_pose_w_;
    Vector6d base_twist_w_;
    
    double ctrl_v_;
    double ctrl_omega_;
    double v_d_;
    double omega_d_;
    
    Quaterniond quaternion_;
    Vector3d euler_old_;
    double old_theta_;
    double time_;
    double dt_;
    int log_counter_;
    
    // Logging
    MatrixXd base_pose_w_log_;
    MatrixXd base_twist_w_log_;
    VectorXd time_log_;
    MatrixXd state_log_;
    MatrixXd des_state_log_;
    
    // Desired state
    double des_x_;
    double des_y_;
    double des_theta_;
    
    // Robot state
    RobotState robot_state_;
    
    // Controller
    std::unique_ptr<LyapunovController> controller_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");
    
    ros::NodeHandle nh("~");
    
    int robot_id = 1;
    bool debug = false;
    
    nh.param("robot_id", robot_id, 1);
    nh.param("debug", debug, false);
    
    std::string robot_name = "limo" + std::to_string(robot_id);
    
    ROS_INFO("[Controller] Starting controller for robot: %s (debug: %s)", 
             robot_name.c_str(), debug ? "true" : "false");
    
    Controller controller(robot_name, debug);
    controller.startController();
    
    return 0;
}