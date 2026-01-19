#ifndef PLANNING_UTILS_H
#define PLANNING_UTILS_H

#include <vector>
#include <cmath>

#include <map_library/map_data_structures.h> 
#include <libraries/roadmap/roadmap_data_structures.h>

namespace PlanningUtils {
    /**
     * @brief Type-conversion utility to translate 'Point' objects into 'Vertex' objects.
     * * This function acts as an adapter between different data structures. In many 
     * geometry systems, 'Point' might be a raw data structure (like from a ROS message 
     * or a file), while 'Vertex' is a specialized class with geometric methods.
     * * @param p The source Point structure containing x and y members.
     * * @return Vertex A new Vertex object initialized with the same coordinates.
     */
    Vertex toVertex(const Point& p);



    /**
     * @brief Checks if a given vertex lies within a polygon defined by a sequence of vertices.
     * * This implementation uses the Ray Casting (Even-Odd) algorithm. It conceptually 
     * shoots a ray from the point to infinity in the positive X-direction and counts 
     * intersections with polygon edges.
     * * @param point   The Vertex (x, y) to be tested.
     * @param polygon A vector of vertices defining the polygon's perimeter in order.
     * * @return true    The point is inside the polygon (Odd number of intersections).
     * @return false   The point is outside the polygon (Even number of intersections).
     * * @note Mathematical Logic:
     * 1. An intersection occurs if the ray's Y-coordinate is between the edge's Y-endpoints.
     * 2. The ray's X-position must be less than the X-coordinate of the point on the edge 
     * at that same Y-level (the ray "crosses" to the right).
     */
    bool pointInPolygon(const Vertex& point, const std::vector<Vertex>& polygon); 



    /**
     * @brief Determines if a point is inside a polygon obstacle using the Ray Casting Algorithm.
     * * This algorithm simulates a horizontal ray starting from the 'point' and extending 
     * to the right (positive X direction). It counts how many times this ray intersects 
     * the edges of the polygon.
     * * - If the number of intersections is ODD, the point is INSIDE.
     * - If the number of intersections is EVEN, the point is OUTSIDE.
     * * @param point    The vertex (x, y) coordinate to test.
     * @param obstacle The polygon obstacle containing a list of points.
     * * @return true    If the point is strictly inside the polygon.
     * @return false   If the point is outside the polygon.
     */
    bool pointInObstacle(const Vertex& point, const Obstacle& obstacle);



    /**
     * @brief Aggregates collision results to determine if a point is inside ANY map obstacle.
     * * This is a convenience wrapper that iterates through the entire obstacle list. 
     * It employs "short-circuit" logic: as soon as one obstacle is found to contain 
     * the point, the function exits immediately.
     * * @param point     The Vertex (x, y) coordinate to test.
     * @param obstacles A vector of all Obstacle objects defined in the map.
     * * @return true     If the point lies within the boundary of at least one obstacle.
     * @return false    If the point is in free space relative to all obstacles.
     */
    bool pointInAnyObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles);



    /**
     * @brief Performs a continuous collision check for a line segment against a set of obstacles.
     * * This function uses a dual-verification strategy:
     * 1. Intersection Check: Detects if the segment crosses any obstacle edge.
     * 2. Containment Check: Detects if the entire segment lies inside an obstacle 
     * * @param p1            The start vertex of the path segment.
     * @param p2            The end vertex of the path segment.
     * @param obstacles     A list of obstacles (polygons) to check against.
     * * @return true         If the segment intersects an edge or is contained within an obstacle.
     * @return false        If the segment is entirely in free space.
     */
    bool lineSegmentIntersectsObstacle(const Vertex& p1, const Vertex& p2, 
                                       const std::vector<Obstacle>& obstacles);



    /**
     * @brief Calculates the intersection point of two line segments AB and CD.
     * * The function converts the segments into the general line equation form: ax + by = c.
     * It then solves the system of two linear equations using the determinant method.
     * * @param A            Start vertex of the first segment.
     * @param B            End vertex of the first segment.
     * @param C            Start vertex of the second segment.
     * @param D            End vertex of the second segment.
     * @param intersection [Out] The vertex where the segments intersect (only updated if returning true).
     * * @return true         If segments intersect at a single point.
     * @return false        If segments are parallel, collinear, or do not overlap.
     */
    bool getSegmentIntersection(const Vertex& A, const Vertex& B, 
                                const Vertex& C, const Vertex& D, 
                                Vertex& intersection);



    /**
     * @brief Calculates the minimum Euclidean distance from a given point to the nearest obstacle.
     * * This function iterates through a collection of obstacles to find the closest point of 
     * approach. It also performs an interior check to handle cases where the point might 
     * be located inside an obstacle.
     * * @param point     The reference Vertex (x, y) from which distance is measured.
     * @param obstacles A vector of Obstacle objects (typically polygons defined by vertices).
     * * @return double   The shortest distance found. Returns 0.0 if the point is inside an obstacle.
     * Returns std::numeric_limits<double>::max() if the obstacle list is empty.
     * * @note Accuracy Note: This specific implementation measures distance to obstacle VERTICES. 
     * It does not currently calculate the perpendicular distance to obstacle EDGES.
     */
    double distanceToNearestObstacle(const Vertex& point, const std::vector<Obstacle>& obstacles);
    
    

    /**
     * @brief Validates if a straight-line path between two vertices is safe and maintains clearance.
     * * This function performs a two-stage safety check:
     * 1. Binary Collision Check: Determines if the line segment directly hits an obstacle.
     * 2. Clearance Sampling: Discretizes the segment into small steps and verifies that every 
     * sampled point maintains a minimum buffer distance (clearance) from the nearest obstacle.
     * * @param p1            The starting Vertex (x, y) of the segment.
     * @param p2            The ending Vertex (x, y) of the segment.
     * @param obstacles     A collection of obstacle geometries to check against.
     * @param min_clearance The minimum allowed distance between any point on the segment and an obstacle.
     * * @return true         If the segment does not intersect any obstacle AND all sampled points 
     * maintain the required clearance.
     * @return false        If a direct intersection occurs OR any sampled point is too close to an obstacle.
     * @warning Tunneling: If obstacles are smaller than the 'step_size' (0.05), it is theoretically 
     * possible for the sampler to "jump over" a clearance violation.
     */
    bool isSegmentSafe(const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles, double min_clearance);



    /**
     * @brief Validates if a specific coordinate (x, y) is within the map boundaries and 
     * satisfies safety clearance requirements from obstacles.
     * * This function follows a "failing fast" logic sequence:
     * 1. Boundary Check: Is the point inside the navigable map area?
     * 2. Precision Handling: Is the clearance requirement negligible?
     * 3. Proximity Check: Does the point maintain the required buffer distance?
     * * @param x             The X-coordinate of the point to check.
     * @param y             The Y-coordinate of the point to check.
     * @param map           The Map object containing border polygons and obstacle data.
     * @param min_clearance The minimum allowed distance to the nearest obstacle.
     * * @return true         If the point is inside the map AND far enough from obstacles.
     * @return false        If the point is outside the map, inside an obstacle, or 
     * within the clearance buffer zone.
     */
    bool isPointValid(double x, double y, const Map& map, double min_clearance = 0.0);
    


    /**
     * @brief Integrates a specific coordinate into the existing roadmap.
     * * This function handles the insertion of a new vertex (e.g., a start or goal position) 
     * into the graph and attempts to establish valid edges to existing nodes using a 
     * strategy for collision checking.
     * * @param roadmap    Shared pointer to the Roadmap graph structure.
     * @param pos        The 2D Vertex coordinates to be integrated.
     * @param obstacles  Vector of obstacle entities for collision validation.
     * @param label      A descriptive string (e.g., "Start", "Goal") for identification.
     */
    void integratePosition( std::shared_ptr<Roadmap>& roadmap, const Vertex& pos, const std::vector<Obstacle>& obstacles, const std::string& label);
    


    /**
     * @brief Optimizes a discrete path by applying a greedy shortcutting algorithm and distance filtering.
     * * This function reduces the complexity of a path generated from a roadmap (like PRM or RRT).
     * It attempts to "pull the string" between non-adjacent waypoints to create straight-line 
     * shortcuts, significantly reducing the total node count and improving path smoothness.
     *
     * @section Algorithm Steps:
     * 1. Greedy Shortcutting: Iterates from the current point to the furthest possible 
     * reachable point in the path. If a direct linear segment is collision-free (safe), 
     * it skips all intermediate nodes.
     * 2. Distance Filtering: Performs a second pass to remove nodes that are too close 
     * to each other (Euclidean distance < 1.0), preventing jitter in the final trajectory.
     * * @param rawPath       A vector of vertex indices representing the initial unoptimized path.
     * @param roadmap       The graph structure containing vertex coordinates for safety checking.
     * @param obstacles     A list of obstacles to check against for shortcut validity.
     * @param SAFETY_MARGIN The minimum clearance distance required from obstacles during shortcutting.
     * * @return std::vector<int> A streamlined path containing only essential waypoints.
    */
    std::vector<int> optimizePath(const std::vector<int>& rawPath, const Roadmap& roadmap, const std::vector<Obstacle>& obstacles, double SAFETY_MARGIN=0.5);
}

#endif // PLANNING_UTILS_H