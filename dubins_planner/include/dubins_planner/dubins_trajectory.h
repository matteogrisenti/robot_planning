#ifndef DUBINS_TRAJECTORY_H
#define DUBINS_TRAJECTORY_H

#include <vector>
#include <cmath>
#include <limits>
#include <array>

namespace Kinematics {

	// Dubins Path Types
    enum class PathType { LSL, RSR, LSR, RSL, RLR, LRL, NONE };

	// Segment structure representing a single Dubins path segment
    struct Segment {
        double start_x, start_y, start_heading;	// Starting pose of the segment
        double curvature, length;				// Curvature and length of the segment
        double end_x, end_y, end_heading;		// Ending pose of the segment
    };

	// Trajectory structure representing a full Dubins path
    struct Trajectory {
        std::array<Segment, 3> segments;	// Three segments of the Dubins path
        double total_length;				// Total length of the path
        PathType type;						// Type of the Dubins path
    };

	// State structure representing a robot pose
    struct State {
        double x, y, yaw;
    };

    class DubinsSolver {
    public:
        DubinsSolver(double max_curvature);

		/**
         * @brief Computes the optimal Dubins path between start and goal states.
         * * The method normalizes the problem, tests the 6 analytical combinations and 
         * selects the one with the minimum path cost.
         * @param start Start state of the robot.
         * @param goal Goal state desired.
         * @param result Reference to the Trajectory structure where the path will be stored.
         * @return true if a valid geometric solution was found.
         */
        bool compute_optimal_path(State start, State goal, Trajectory& result);
        
		/**
         * @brief Interpolates the trajectory in a series of discrete points for visualization or control.
         * @param trajectory The computed trajectory to interpolate.
         * @param samples_per_seg Number of samples to generate per segment.
         * @return Vector of sampled states along the path.
         */
        static std::vector<State> interpolate(const Trajectory& trajectory, int samples_per_seg);

		/**
         * @brief Projects a state forward along a line or arc.
         * * Core function that integrates the robot's kinematics starting from (x0, y0, yaw0) 
         * for a distance 'dist' given a curvature 'k'.
         * * @param dist Distance to travel.
         * @param x0, y0, yaw0 Initial pose.
         * @param k Applied curvature.
         * @param out Stato risultante dopo la proiezione.
         */
        static void project_state(double dist, double x0, double y0, double yaw0, double k, State& out);

    private:
        double _max_k;

        // Core Path Primitives
        bool solve_LSL(double d_th0, double d_thf, double k_max, double s[3]);
        bool solve_RSR(double d_th0, double d_thf, double k_max, double s[3]);
        bool solve_LSR(double d_th0, double d_thf, double k_max, double s[3]);
        bool solve_RSL(double d_th0, double d_thf, double k_max, double s[3]);
        bool solve_RLR(double d_th0, double d_thf, double k_max, double s[3]);
        bool solve_LRL(double d_th0, double d_thf, double k_max, double s[3]);

        // Internal Utilities
        static double normalize_angle(double angle);
    };
}

#endif