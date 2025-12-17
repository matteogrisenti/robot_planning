#include <ros/ros.h>
#include <string>
#include "robot_control/ros_controller_node.h"

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
    
    RosControllerNode controller(robot_name, debug);
    controller.startController();
    
    return 0;
}