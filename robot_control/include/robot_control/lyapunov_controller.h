/* Lyapunov controller header file 
*    This implements a Lyapunov-based controller for unicycle robots */

#ifndef LYAPUNOV_CONTROLLER_H
#define LYAPUNOV_CONTROLLER_H

#include <cmath>
#include <utility>
#include <iostream>

// Robot state structure
struct RobotState {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

// Lyapunov controller parameters
struct LyapunovParams {
    double K_P;     // Proportional gain
    double K_THETA; // Heading gain
    double DT;      // Time step

    // Constructor
    LyapunovParams(double kp, double kth, double dt) 
        : K_P(kp), K_THETA(kth), DT(dt) {}
};

class LyapunovController {
public:
    explicit LyapunovController(const LyapunovParams& params);

    /** 
     * @brief Main control function for unicycle robot
     * @param robot_state Current state of the robot
     * @param time Current time
     * @param des_x Desired x position
     * @param des_y Desired y position
     * @param des_theta Desired heading angle
     * @param v_d Desired linear velocity
     * @param omega_d Desired angular velocity
     * @param verbose If true, prints debug information
     * @return Pair of control commands: (linear velocity, angular velocity)
     */
    std::pair<double, double> controlUnicycle(
        const RobotState& robot_state,
        double time,
        double des_x, double des_y, double des_theta,
        double v_d, double omega_d,
        bool verbose = false);

    // Math utilities (static to allow access without instantiation)
    static double unwrapAngle(double angle, double old_angle);

private:
    // Controller parameters
    LyapunovParams params_;
};

#endif // LYAPUNOV_CONTROLLER_H