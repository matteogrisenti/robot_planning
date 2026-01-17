#include "dubins_planner/dubins_trajectory.h"

namespace Kinematics {

    DubinsSolver::DubinsSolver(double max_curvature) : _max_k(max_curvature) {}

    double DubinsSolver::normalize_angle(double angle) {
        double res = std::fmod(angle, 2.0 * M_PI);
        if (res < 0) res += 2.0 * M_PI;
        return res;
    }

    void DubinsSolver::project_state(double s, double x0, double y0, double yaw0, double k, State& out) {
        if (std::abs(k) < 1e-9) {
            out.x = x0 + s * std::cos(yaw0);
            out.y = y0 + s * std::sin(yaw0);
        } else {
            out.x = x0 + (std::sin(yaw0 + k * s) - std::sin(yaw0)) / k;
            out.y = y0 + (std::cos(yaw0) - std::cos(yaw0 + k * s)) / k;
        }
        out.yaw = normalize_angle(yaw0 + k * s);
    }

    // --- SOLVERS (Normalizzati con k=1 per stabilità) ---

    bool DubinsSolver::solve_LSL(double th0, double thf, double d, double s[3]) {
        double tmp = std::atan2(std::cos(thf) - std::cos(th0), d + std::sin(th0) - std::sin(thf));
        s[0] = normalize_angle(tmp - th0);
        s[1] = std::sqrt(2.0 + d*d - 2.0*std::cos(th0 - thf) + 2.0*d*(std::sin(th0) - std::sin(thf)));
        s[2] = normalize_angle(thf - tmp);
        return true;
    }

    bool DubinsSolver::solve_RSR(double th0, double thf, double d, double s[3]) {
        double tmp = std::atan2(std::cos(th0) - std::cos(thf), d - std::sin(th0) + std::sin(thf));
        s[0] = normalize_angle(th0 - tmp);
        s[1] = std::sqrt(2.0 + d*d - 2.0*std::cos(th0 - thf) - 2.0*d*(std::sin(th0) - std::sin(thf)));
        s[2] = normalize_angle(tmp - thf);
        return true;
    }

    bool DubinsSolver::solve_LSR(double th0, double thf, double d, double s[3]) {
        double p_sq = -2.0 + d*d + 2.0*std::cos(th0 - thf) + 2.0*d*(std::sin(th0) + std::sin(thf));
        if (p_sq < 0) return false;
        double p = std::sqrt(p_sq);
        double tmp = std::atan2(-std::cos(th0) - std::cos(thf), d + std::sin(th0) + std::sin(thf)) - std::atan2(-2.0, p);
        s[0] = normalize_angle(tmp - th0);
        s[1] = p;
        s[2] = normalize_angle(tmp - thf);
        return true;
    }

    bool DubinsSolver::solve_RSL(double th0, double thf, double d, double s[3]) {
        double p_sq = -2.0 + d*d + 2.0*std::cos(th0 - thf) - 2.0*d*(std::sin(th0) + std::sin(thf));
        if (p_sq < 0) return false;
        double p = std::sqrt(p_sq);
        double tmp = std::atan2(std::cos(th0) + std::cos(thf), d - std::sin(th0) - std::sin(thf)) - std::atan2(2.0, p);
        s[0] = normalize_angle(th0 - tmp);
        s[1] = p;
        s[2] = normalize_angle(thf - tmp);
        return true;
    }

    bool DubinsSolver::solve_RLR(double th0, double thf, double d, double s[3]) {
        double tmp = (6.0 - d*d + 2.0*std::cos(th0 - thf) + 2.0*d*(std::sin(th0) - std::sin(thf))) / 8.0;
        if (std::abs(tmp) > 1.0) return false;
        s[1] = normalize_angle(2.0 * M_PI - std::acos(tmp));
        s[0] = normalize_angle(th0 - std::atan2(std::cos(th0) - std::cos(thf), d - std::sin(th0) + std::sin(thf)) + s[1]/2.0);
        s[2] = normalize_angle(th0 - thf - s[0] + s[1]);
        return true;
    }

    bool DubinsSolver::solve_LRL(double th0, double thf, double d, double s[3]) {
        double tmp = (6.0 - d*d + 2.0*std::cos(th0 - thf) - 2.0*d*(std::sin(th0) - std::sin(thf))) / 8.0;
        if (std::abs(tmp) > 1.0) return false;
        s[1] = normalize_angle(2.0 * M_PI - std::acos(tmp));
        s[0] = normalize_angle(std::atan2(std::cos(thf) - std::cos(th0), d + std::sin(th0) - std::sin(thf)) - th0 + s[1]/2.0);
        s[2] = normalize_angle(thf - th0 - s[0] + s[1]);
        return true;
    }

    bool DubinsSolver::compute_optimal_path(State start, State goal, Trajectory& result) {
        double dx = goal.x - start.x;
        double dy = goal.y - start.y;
        double d_real = std::hypot(dx, dy);
        double theta = std::atan2(dy, dx);

        double alpha = normalize_angle(start.yaw - theta);
        double beta = normalize_angle(goal.yaw - theta);
        
        // Raggio di curvatura rho = 1/k
        double rho = 1.0 / _max_k;
        double d = d_real / rho; // Distanza normalizzata rispetto al raggio

        double best_l = std::numeric_limits<double>::max();
        int best_idx = -1;
        double lengths[6][3];
        bool valid[6];

        valid[0] = solve_LSL(alpha, beta, d, lengths[0]);
        valid[1] = solve_RSR(alpha, beta, d, lengths[1]);
        valid[2] = solve_LSR(alpha, beta, d, lengths[2]);
        valid[3] = solve_RSL(alpha, beta, d, lengths[3]);
        valid[4] = solve_RLR(alpha, beta, d, lengths[4]);
        valid[5] = solve_LRL(alpha, beta, d, lengths[5]);

        for (int i = 0; i < 6; ++i) {
            if (valid[i]) {
                double total = lengths[i][0] + lengths[i][1] + lengths[i][2];
                if (total < best_l) {
                    best_l = total;
                    best_idx = i;
                }
            }
        }

        if (best_idx == -1) return false;

        // Converti le lunghezze normalizzate in metri
        double seg_lens[3] = { lengths[best_idx][0] * rho, 
                               lengths[best_idx][1] * rho, 
                               lengths[best_idx][2] * rho };

        static const double k_signs[6][3] = {
            {1,0,1}, {-1,0,-1}, {1,0,-1}, {-1,0,1}, {-1,1,-1}, {1,-1,1}
        };

        double cur_x = start.x, cur_y = start.y, cur_yaw = start.yaw;

        for (int i = 0; i < 3; ++i) {
            result.segments[i].start_x = cur_x;
            result.segments[i].start_y = cur_y;
            result.segments[i].start_heading = cur_yaw;
            result.segments[i].curvature = k_signs[best_idx][i] * _max_k;
            result.segments[i].length = seg_lens[i];
            
            State next;
            // FORZATURA FINALE: L'ultimo segmento deve finire esattamente sul goal
            if (i == 2) {
                next.x = goal.x;
                next.y = goal.y;
                next.yaw = goal.yaw;
            } else {
                project_state(seg_lens[i], cur_x, cur_y, cur_yaw, result.segments[i].curvature, next);
            }
            
            result.segments[i].end_x = next.x;
            result.segments[i].end_y = next.y;
            result.segments[i].end_heading = next.yaw;
            
            cur_x = next.x; cur_y = next.y; cur_yaw = next.yaw;
        }

        result.total_length = seg_lens[0] + seg_lens[1] + seg_lens[2];
        result.type = static_cast<PathType>(best_idx);
        return true;
    }

    std::vector<State> DubinsSolver::interpolate(const Trajectory& trajectory, int samples) {
        std::vector<State> path;
        for (const auto& seg : trajectory.segments) {
            for (int i = 0; i <= samples; ++i) {
                double step = (seg.length / (double)samples) * i;
                State p;
                project_state(step, seg.start_x, seg.start_y, seg.start_heading, seg.curvature, p);
                path.push_back(p);
            }
        }
        return path;
    }

} // namespace Kinematics