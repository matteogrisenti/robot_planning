#include "robot_control/lyapunov_controller.h"
#include <cmath>
#include <iostream>

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
    // Questo è il passaggio mancante fondamentale per correggere lo "slittamento"
    double c = cos(robot_state.theta);
    double s = sin(robot_state.theta);

    // e_x_body: errore longitudinale (lungo la direzione di marcia)
    double e_x_body = c * ex_w + s * ey_w;
    
    // e_y_body: errore laterale (cross-track error)
    double e_y_body = -s * ex_w + c * ey_w;

    // 3. Legge di Controllo (Trajectory Tracking)
    // Usa Feedforward (v_d, omega_d) + Feedback sugli errori Body Frame
    
    // Controllo Velocità Lineare:
    // v_d * cos(e_theta) riduce la velocità se siamo storti
    // K_P * e_x_body corregge la posizione longitudinale
    double ctrl_v = v_d * cos(e_theta) + params_.K_P * e_x_body;

    // Controllo Velocità Angolare:
    // omega_d: feedforward dalla pianificazione (curva prevista)
    // K_THETA * sin(e_theta): corregge l'orientamento
    // K_LAT * e_y_body: corregge l'errore laterale (la "deriva")
    // Usiamo params_.K_THETA come base per il guadagno laterale per semplicità
    // Un guadagno extra per e_y aiuta a rientrare in traiettoria velocemente.
    double ky = params_.K_THETA; // O un valore più alto se serve, es. 3.0
    
    double ctrl_omega = omega_d + params_.K_THETA * sin(e_theta) + ky * e_y_body;

    if (verbose) {
        std::cout << "e_x_b: " << e_x_body << ", e_y_b: " << e_y_body << ", e_theta: " << e_theta << std::endl;
        std::cout << "ctrl_v: " << ctrl_v << ", ctrl_omega: " << ctrl_omega << std::endl;
    }

    return {ctrl_v, ctrl_omega};
}