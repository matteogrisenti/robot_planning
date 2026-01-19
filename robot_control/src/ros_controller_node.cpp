#include "robot_control/ros_controller_node.h" // .h library
#include <cmath> // per std::isnan

/* Implementation of the RosControllerNode class */
// Constructor
RosControllerNode::RosControllerNode(const std::string& robot_name, bool debug)
    : robot_name_(robot_name), debug_(debug), nh_("~") 
{
    ROS_INFO("[Controller] Controller initialized for robot: %s", robot_name_.c_str());
}
// Destructor
RosControllerNode::~RosControllerNode() {
    sendCommands(0.0, 0.0);
    if (debug_) {
        plotData();
    }
}

// Main control loop
void RosControllerNode::startController() {
    initVars();                     // Initialize variables                 
    startPublisherSubscribers();    // Start publishers and subscribers 
    
    // Default gains
    double k_p = 2.0;    // Proportional gain           
    double k_th = 4.0;   // Heading gain
    double dt = 0.01;    // Time step
    
    nh_.param("k_p", k_p, 2.0);
    nh_.param("k_th", k_th, 4.0);
    nh_.param("dt", dt, 0.01);
    
    // Initialize Lyapunov Controller
    LyapunovParams params(k_p, k_th, dt);   
    controller_ = std::make_unique<LyapunovController>(params); 
    
    dt_ = dt;
    ros::Rate rate(1.0 / dt);   // Control loop rate
    
    ROS_INFO("[Controller] Starting control loop at %.1f Hz", 1.0/dt);
    
    // Control loop
    while (ros::ok()) {
        try {
            robot_state_.x = base_pose_w_(0);       // Received Current x position
            robot_state_.y = base_pose_w_(1);       // Received Current y position
            robot_state_.theta = base_pose_w_(5);   // Received Current heading angle
            
            // Unwrap desired theta
            des_theta_ = LyapunovController::unwrapAngle(des_theta_, old_theta_);
            old_theta_ = des_theta_;
            
            // Compute control commands
            std::pair<double, double> control = controller_->controlUnicycle(
                robot_state_, time_,
                des_x_, des_y_, des_theta_,
                v_d_, omega_d_,
                false 
            );
            
            // Extract control commands
            ctrl_v_ = control.first;
            ctrl_omega_ = control.second;
            
            // --- SAFETY: NAN CHECK ---
            if (std::isnan(ctrl_v_) || std::isnan(ctrl_omega_) || std::isinf(ctrl_v_) || std::isinf(ctrl_omega_)) {
                // Se riceviamo NaN (dal Dubins planner rotto), fermiamo il robot invece di mandare comandi invalidi
                ctrl_v_ = 0.0;
                ctrl_omega_ = 0.0;
                // ROS_WARN_THROTTLE(1.0, "[Controller] NaN detected! Stopping robot.");
            }

            // Send commands to robot
            sendCommands(ctrl_v_, ctrl_omega_);
            logData();  // Log data for analysis
            
            // Spin and sleep
            ros::spinOnce();
            rate.sleep();
            
            // Update time
            time_ += dt_;
            time_ = std::round(time_ * 10000.0) / 10000.0; 
            
        } catch (const ros::Exception& e) {
            ROS_ERROR("[Controller] ROS Exception: %s", e.what());
            sendCommands(0.0, 0.0);
            break;
        }
    }
}

// Initialization methods
void RosControllerNode::initVars() {
    base_pose_w_.setZero();     // Initialize base pose to zero
    base_twist_w_.setZero();    // Initialize base twist to zero
    ctrl_v_ = 0.0;              // Control linear velocity
    ctrl_omega_ = 0.0;          // Control angular velocity
    v_d_ = 0.0;                 // Desired linear velocity
    omega_d_ = 0.0;             // Desired angular velocity 
    quaternion_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);  
    euler_old_.setZero();       // Initialize euler angles to zero
    old_theta_ = 0.0;           // Previous desired heading angle
    time_ = 0.0;
    log_counter_ = 0;
    
    int buffer_size = 10000;    // Default buffer size for logging
    nh_.param("buffer_size", buffer_size, 10000);   
    
    // Initialize logging data structures
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
    
    des_x_ = 0.0;
    des_y_ = 0.0;
    des_theta_ = 0.0;
    
    ROS_INFO("[Controller] Variables initialized");
}

// Start publishers and subscribers ROS nodes
void RosControllerNode::startPublisherSubscribers() {
    ros::NodeHandle nh_robot("/" + robot_name_);

    // Publisher for velocity commands
    command_pub_ = nh_robot.advertise<geometry_msgs::Twist>("cmd_vel", 1);  
    
    // Subscribers for odometry and reference
    odom_sub_ = nh_robot.subscribe("odom", 1, &RosControllerNode::receivePose, this, ros::TransportHints().tcpNoDelay());
    ref_sub_ = nh_robot.subscribe("ref", 1, &RosControllerNode::receiveReference, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO("[Controller] Publishers and subscribers started");
}

// Callbacks and Helpers
// Unwrap vector of angles
Eigen::Vector3d RosControllerNode::unwrapVector(const Eigen::Vector3d& vec, const Eigen::Vector3d& old_vec) {
    Eigen::Vector3d result;
    for (int i = 0; i < 3; ++i) {
        result(i) = LyapunovController::unwrapAngle(vec(i), old_vec(i));
    }
    return result;
}

// Callback to receive robot pose
void RosControllerNode::receivePose(const nav_msgs::Odometry::ConstPtr& msg) {
    quaternion_ = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    Eigen::Vector3d euler(roll, pitch, yaw);
    euler = unwrapVector(euler, euler_old_);
    euler_old_ = euler;
    base_pose_w_(0) = msg->pose.pose.position.x;
    base_pose_w_(1) = msg->pose.pose.position.y;
    base_pose_w_(2) = msg->pose.pose.position.z;
    base_pose_w_(3) = euler(0);
    base_pose_w_(4) = euler(1);
    base_pose_w_(5) = euler(2);
    base_twist_w_(0) = msg->twist.twist.linear.x;
    base_twist_w_(1) = msg->twist.twist.linear.y;
    base_twist_w_(2) = msg->twist.twist.linear.z;
    base_twist_w_(3) = msg->twist.twist.angular.x;
    base_twist_w_(4) = msg->twist.twist.angular.y;
    base_twist_w_(5) = msg->twist.twist.angular.z;
}

void RosControllerNode::receiveReference(const robot_control::Reference::ConstPtr& msg) {
    if (msg->plan_finished) {
        // ROS_INFO("[Controller] Controller finish, plotting data...");
        plotData();
    } else {    
        des_x_ = msg->x_d;
        des_y_ = msg->y_d;
        des_theta_ = msg->theta_d;
        v_d_ = msg->v_d;
        omega_d_ = msg->omega_d;
    }
}

void RosControllerNode::sendCommands(double lin_vel, double ang_vel) {
    geometry_msgs::Twist msg;
    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;
    command_pub_.publish(msg);
}

void RosControllerNode::logData() {
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

void RosControllerNode::plotData() {
    // FIX: Scrivi in /tmp per evitare errori di permessi/path
    std::string filename = "/tmp/" + robot_name_ + "_trajectory_data.txt";
    std::ofstream file(filename);
    if (file.is_open()) {
        file << "# time x y theta des_x des_y des_theta\n";
        for (int i = 0; i < log_counter_; ++i) {
            file << time_log_(i) << " " << state_log_(0, i) << " " << state_log_(1, i) << " " << state_log_(2, i) << " " << des_state_log_(0, i) << " " << des_state_log_(1, i) << " " << des_state_log_(2, i) << "\n";
        }
        file.close();
        ROS_INFO("[Controller] Data saved to %s", filename.c_str());
    }
}