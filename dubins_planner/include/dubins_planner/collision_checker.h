#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "map_library/map_data_structures.h"
#include "dubins_planner/dubins_trajectory.h"

// Point 2D
struct Point2D {
    double x;
    double y;
};

// Obstacle Circle
struct CircleObstacle {
    Point2D center;
    double radius;
};

// Obstacle Polygon
struct PolygonObstacle {
    std::vector<Point2D> vertices; 
    Point2D centroid;              
    double bounding_radius;        
};

class CollisionChecker {
public:
    CollisionChecker() = default;
    /**
     * @brief Constructor.
     * @param robot_radius The radius of the robot.
     * @param safety_margin The safety margin (optional, default 0).
     */
    CollisionChecker(double robot_radius, double safety_margin = 0.0);

    /**
     * @brief Main collision check function.
     * @param robot_pose The position of the robot (x,y).
     * @return true if in collision, false otherwise.
     */
    [[nodiscard]] bool check(const Point2D& robot_pose) const;

    /**
     * @brief Converts obstacles and borders into the optimized format for the CollisionChecker.
     */
    void update_collision_cache(const Map& map);

    /**
     * @brief Checks if a single configuration (x,y) is valid (free).
     */
    bool is_state_valid(double x, double y) const;

    /**
     * @brief Checks if a planned Dubins trajectory is free of collisions.
     * @param trajectory The planned path from the DubinsSolver.
     * @return true if the path is clear, false if any point collides.
     */
    bool is_dubins_path_valid(const Kinematics::Trajectory& trajectory) const;

    
private:
    // The total effective radius (robot + margin)
    double effective_robot_radius_;
    // The square of the effective radius (for optimization comparisons)
    double effective_radius_sq_;

    // Optimized cache (Polygons and Circles ready for checking)
    std::vector<CircleObstacle> cached_circles_;
    std::vector<PolygonObstacle> cached_polygons_;

    // --- Private Helper Functions ---

    // Helper for squared distance between two points
    [[nodiscard]] double distSq(const Point2D& p1, const Point2D& p2) const {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return dx * dx + dy * dy;
    }

    // Helper for squared distance from a point to a segment
    double distToSegmentSquared(const Point2D& p, const Point2D& a, const Point2D& b) const;

    // Helper for checking collision with a single polygon using hybrid strategy
    bool checkSinglePolygon(const Point2D& robot_pose, const PolygonObstacle& poly) const;
};

#endif 
