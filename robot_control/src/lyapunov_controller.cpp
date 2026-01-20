#include "robot_control/lyapunov_controller.h"
#include <cmath>
#include <iostream>
#include <algorithm> 

LyapunovController::LyapunovController(const LyapunovParams& params) 
    : params_(params) {}

double LyapunovController::unwrapAngle(double angle, double old_angle) {
    double diff = angle - old_angle;
    while (diff > M_PI) { 
        angle -= 2.0 * M_PI; 
        diff = angle - old_angle; 
    }
    while (diff < -M_PI) { 
        angle += 2.0 * M_PI; 
        diff = angle - old_angle; 
    }
    return angle;
}

std::pair<double, double> LyapunovController::controlUnicycle(
    const RobotState& robot_state,
    double time,
    double des_x, double des_y, double des_theta,
    double v_d, double omega_d,
    bool verbose)
{
    // 1. Calculate Errors in World Frame
    double ex_w = des_x - robot_state.x;
    double ey_w = des_y - robot_state.y;
    
    // Normalize angular error
    double e_theta = des_theta - robot_state.theta;
    e_theta = atan2(sin(e_theta), cos(e_theta)); 

    // 2. Project Errors into Robot Body Frame
    double c = cos(robot_state.theta);
    double s = sin(robot_state.theta);

    // e_x_body: longitudinal error
    double e_x_body = c * ex_w + s * ey_w;
    
    // e_y_body: lateral error
    double e_y_body = -s * ex_w + c * ey_w;

    // 3. CONTROL LAW APPLICATION
    
    // --- LINEAR VELOCITY CONTROL (Modified for Constant Velocity) ---
    // Standard Lyapunov would be: v_cmd = v_d * cos(e_theta) + K_P * e_x_body;
    // This causes the robot to slow down significantly on turns or when orientation error exists.
    //
    // FIX: To satisfy the "Constant Velocity" assumption, we push v_d directly.
    // We only stop if the orientation error is extreme (> 90 deg) to prevent moving backwards blind.
    
    double ctrl_v = v_d; 
    
    // Safety check: if we are facing the wrong way (>90 deg error), stop to turn in place (or wait for omega to fix it).
    // This usually shouldn't happen with a good Dubins planner.
    if (std::abs(e_theta) > M_PI_2) {
        ctrl_v = 0.0; 
    }

    // --- ANGULAR VELOCITY CONTROL (Standard Lyapunov) ---
    // This part remains active to steer the robot towards the path.
    // omega = omega_d + K_theta * sin(e_theta) + K_y * e_y * sinc(e_theta)
    // We use a simplified form where K_P acts on the lateral error.
    
    double ctrl_omega = omega_d + params_.K_THETA * sin(e_theta) + params_.K_P * e_y_body;

    if (verbose) {
        std::cout << "[Lyapunov] e_x: " << e_x_body 
                  << " e_y: " << e_y_body 
                  << " e_th: " << e_theta 
                  << " -> v_out: " << ctrl_v << std::endl;
    }

    return {ctrl_v, ctrl_omega};
}