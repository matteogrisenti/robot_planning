#include "robot_control/lyapunov_controller.h"

LyapunovController::LyapunovController(const LyapunovParams& params) 
    : params_(params) {}

double LyapunovController::unwrapAngle(double angle, double old_angle) {
    double diff = angle - old_angle;
    if (diff > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (diff < -M_PI) {
        angle += 2.0 * M_PI;
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
        std::cout << "e_x: " << e_x << ", e_y: " << e_y << ", e_theta: " << e_theta << std::endl;
        std::cout << "ctrl_v: " << ctrl_v << ", ctrl_omega: " << ctrl_omega << std::endl;
    }

    return {ctrl_v, ctrl_omega};
}