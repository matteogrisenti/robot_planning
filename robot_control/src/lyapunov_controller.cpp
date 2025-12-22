#include "robot_control/lyapunov_controller.h"
#include <cmath>
#include <iostream>
#include <algorithm> // Per std::max

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
    // 1. Calcolo Errori nel World Frame
    double ex_w = des_x - robot_state.x;
    double ey_w = des_y - robot_state.y;
    
    // Normalizza errore angolare
    double e_theta = des_theta - robot_state.theta;
    e_theta = atan2(sin(e_theta), cos(e_theta)); 

    // 2. Proiezione Errori nel Body Frame del Robot
    double c = cos(robot_state.theta);
    double s = sin(robot_state.theta);

    // e_x_body: errore longitudinale
    double e_x_body = c * ex_w + s * ey_w;
    
    // e_y_body: errore laterale
    double e_y_body = -s * ex_w + c * ey_w;

    // 3. Legge di Controllo
    
    // CORREZIONE RETROMARCIA:
    // Il termine feedforward (v_d * cos) spinge avanti.
    // Il termine feedback (K_P * e_x_body) corregge la posizione.
    double raw_v = v_d * cos(e_theta) + params_.K_P * e_x_body;

    // FIX: Impediamo velocità negative. 
    // Se il robot è davanti al target (raw_v < 0), si ferma (0.0) e aspetta, non torna indietro.
    // Questo è cruciale per i veicoli Dubins che non dovrebbero fare manovre in reverse durante il path following.
    double ctrl_v = std::max(0.0, raw_v);

    // Controllo Velocità Angolare (rimane invariato con correzione laterale)
    double ky = params_.K_THETA; 
    double ctrl_omega = omega_d + params_.K_THETA * sin(e_theta) + ky * e_y_body;

    if (verbose) {
        std::cout << "e_x: " << e_x_body << " e_y: " << e_y_body << " v_raw: " << raw_v << " v_out: " << ctrl_v << std::endl;
    }

    return {ctrl_v, ctrl_omega};
}