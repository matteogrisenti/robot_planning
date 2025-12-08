#include "roadmap/combinatorial_planning.h"

#include <queue>
#include <cmath>
#include <limits>


// =============================================================================
// EXACT CELL DECOMPOSITION (Trapezoidal Decomposition)
// =============================================================================
std::shared_ptr<Roadmap> CombinatorialPlanning::exactCellDecomposition(const Map& map) {
    auto roadmap = std::make_shared<Roadmap>();
    
    // Compute trapezoidal decomposition
    std::vector<Trapezoid> trapezoids = computeTrapezoidalDecomposition(map);
    // Link it to the roadmap for plotting
    roadmap->debugTrapezoids = std::make_shared<std::vector<Trapezoid>>(trapezoids);
    
    if (trapezoids.empty()) {
        return roadmap;
    }
    
    // Create vertices at trapezoid centers
    std::vector<int> vertexIndices;
    for (const auto& trap : trapezoids) {
        int idx = roadmap->addVertex(trap.center);
        vertexIndices.push_back(idx);
    }
    
    // Connect adjacent trapezoids
    connectAdjacentTrapezoids(trapezoids);
    
    // Add edges between adjacent trapezoid centers
    for (size_t i = 0; i < trapezoids.size(); i++) {
        for (int neighborIdx : trapezoids[i].neighbors) {
            if (neighborIdx >= 0 && neighborIdx < vertexIndices.size()) {
                roadmap->addEdge(vertexIndices[i], vertexIndices[neighborIdx], false);
            }
        }
    }
    
    return roadmap;
}

std::vector<Trapezoid> CombinatorialPlanning::computeTrapezoidalDecomposition(const Map& map) {
    std::vector<Trapezoid> trapezoids;
    
    // 1. Collect Critical X Coordinates
    std::set<double> criticalX;

    // --- FIX START: FORCE MAP BOUNDS ---
    // Explicitly get the global bounding box of the map to ensure we cover the FULL width
    double mapMinX, mapMinY, mapMaxX, mapMaxY;
    map.get_bounding_box(mapMinX, mapMinY, mapMaxX, mapMaxY);
    
    criticalX.insert(mapMinX);
    criticalX.insert(mapMaxX);
    // --- FIX END ---

    // Add polygon vertices just in case the bounding box is looser than the polygon
    const std::vector<Point>& mapBoundary = map.borders.get_points(); 
    for (const auto& p : mapBoundary) {
        criticalX.insert(p.x);
    }

    // Add Obstacle Vertices
    for (const auto& obs : map.obstacles.get_obstacles()) {
        for (const auto& p : obs.get_points()) {
            // Only consider points strictly inside the map bounds
            if (p.x > mapMinX && p.x < mapMaxX) {
                criticalX.insert(p.x);
            }
        }
    }

    std::vector<double> xCoords(criticalX.begin(), criticalX.end());

    // 2. Iterate through vertical strips
    for (size_t i = 0; i < xCoords.size() - 1; i++) {
        double x1 = xCoords[i];
        double x2 = xCoords[i + 1];
        double xMid = (x1 + x2) / 2.0;

        // Skip negligible strips
        if (x2 - x1 < 1e-6) continue;

        // A. CALCULATE MAP "CEILING" AND "FLOOR" (SLANTED)
        // We compute the allowed Y range at the Left (x1) and Right (x2) sides.
        std::pair<double, double> mapRangeLeft = getMapYBoundsAtX(x1, mapBoundary);
        std::pair<double, double> mapRangeRight = getMapYBoundsAtX(x2, mapBoundary);

        // B. COLLECT OBSTACLE EVENTS AT xMid
        // We still treat obstacles as flat slabs within the strip for simplicity, 
        // but we will clip them against the slanted map walls.
        std::vector<double> yCuts;
        
        // Add the map bounds themselves as cuts (projected to mid for sorting)
        double midMapFloor = (mapRangeLeft.first + mapRangeRight.first) / 2.0;
        double midMapCeil = (mapRangeLeft.second + mapRangeRight.second) / 2.0;
        yCuts.push_back(midMapFloor);
        yCuts.push_back(midMapCeil);

        for (const auto& obstacle : map.obstacles.get_obstacles()) {
            double obsMinX, obsMinY, obsMaxX, obsMaxY;
            obstacle.get_bounding_box(obsMinX, obsMinY, obsMaxX, obsMaxY);

            // If obstacle overlaps this X-strip
            if (obsMaxX > x1 && obsMinX < x2) {
                // Clamp obstacle Ys to stay inside map Y-range (at midpoint)
                double clampedMin = std::max(obsMinY, midMapFloor);
                double clampedMax = std::min(obsMaxY, midMapCeil);
                
                if (clampedMin < clampedMax) {
                    yCuts.push_back(clampedMin);
                    yCuts.push_back(clampedMax);
                }
            }
        }

        std::sort(yCuts.begin(), yCuts.end());
        auto last = std::unique(yCuts.begin(), yCuts.end());
        yCuts.erase(last, yCuts.end());

        // C. CONSTRUCT TRAPEZOIDS
        for (size_t j = 0; j < yCuts.size() - 1; j++) {
            // These are the "flat" Y candidates based on obstacles
            double yBottomCandidate = yCuts[j];
            double yTopCandidate = yCuts[j+1];
            
            // Check if this candidate strip is free (at center)
            double yMid = (yBottomCandidate + yTopCandidate) / 2.0;
            Vertex center(xMid, yMid);

            if (!pointInAnyObstacle(center, map.obstacles.get_obstacles())) {
                
                // D. CLIP AGAINST SLANTED MAP BOUNDARIES (The Logic Fix)
                // We define the trapezoid by intersecting the "Flat Slab" (yBottom->yTop)
                // with the "Slanted Map Tunnel" (mapRangeLeft -> mapRangeRight).
                
                // 1. Calculate the map's floor/ceil at x1 and x2
                // (Already have mapRangeLeft, mapRangeRight)

                // 2. The trapezoid coordinates are the TIGHTEST bounds
                double tly = std::min(yTopCandidate, mapRangeLeft.second);  // Top-Left Y
                double try_ = std::min(yTopCandidate, mapRangeRight.second); // Top-Right Y
                
                double bly = std::max(yBottomCandidate, mapRangeLeft.first); // Bottom-Left Y
                double bry = std::max(yBottomCandidate, mapRangeRight.first); // Bottom-Right Y

                // Validity Check: Ensure top is actually above bottom
                if (tly > bly + 1e-6 && try_ > bry + 1e-6) {
                    trapezoids.emplace_back(x1, x2, tly, try_, bly, bry);
                }
            }
        }
    }
    
    return trapezoids;
}

void CombinatorialPlanning::connectAdjacentTrapezoids(std::vector<Trapezoid>& trapezoids) {
    const double epsilon = 1e-5;
    
    for (size_t i = 0; i < trapezoids.size(); i++) {
        for (size_t j = i + 1; j < trapezoids.size(); j++) {
            // Check vertical adjacency (Right of T1 == Left of T2)
            // Or (Left of T1 == Right of T2)
            bool t1_touch_t2 = std::abs(trapezoids[i].rightX - trapezoids[j].leftX) < epsilon;
            bool t2_touch_t1 = std::abs(trapezoids[i].leftX - trapezoids[j].rightX) < epsilon;

            if (!t1_touch_t2 && !t2_touch_t1) continue;

            // Check vertical overlap
            // We compare the vertical interval at the shared boundary X.
            // If T1 touches T2, the shared X is T1.rightX (which is approx T2.leftX).
            
            double i_top, i_bot, j_top, j_bot;

            if (t1_touch_t2) {
                // Shared boundary is right side of i, left side of j
                i_top = trapezoids[i].topRightY;
                i_bot = trapezoids[i].bottomRightY;
                j_top = trapezoids[j].topLeftY;
                j_bot = trapezoids[j].bottomLeftY;
            } else { // t2_touch_t1
                // Shared boundary is left side of i, right side of j
                i_top = trapezoids[i].topLeftY;
                i_bot = trapezoids[i].bottomLeftY;
                j_top = trapezoids[j].topRightY;
                j_bot = trapezoids[j].bottomRightY;
            }

            // Interval Overlap Check: [i_bot, i_top] overlaps [j_bot, j_top]
            bool overlap = std::max(i_bot, j_bot) < std::min(i_top, j_top) - epsilon;

            if (overlap) {
                trapezoids[i].neighbors.push_back(j);
                trapezoids[j].neighbors.push_back(i);
            }
        }
    }
}







// =============================================================================
// GEOMETRIC HELPER FUNCTIONS
// =============================================================================
bool CombinatorialPlanning::pointInObstacle(const Vertex& point, const Obstacle& obstacle) {
    std::vector<Point> polygon = obstacle.get_points();
    int n = polygon.size();
    bool inside = false;
    
    for (int i = 0, j = n - 1; i < n; j = i++) {
        double xi = polygon[i].x, yi = polygon[i].y;
        double xj = polygon[j].x, yj = polygon[j].y;
        
        bool intersect = ((yi > point.y) != (yj > point.y)) &&
                        (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    
    return inside;
}

bool CombinatorialPlanning::pointInAnyObstacle(const Vertex& point, 
                                               const std::vector<Obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        if (pointInObstacle(point, obstacle)) {
            return true;
        }
    }
    return false;
}

bool CombinatorialPlanning::lineSegmentIntersectsObstacle(
    const Vertex& p1, const Vertex& p2, const std::vector<Obstacle>& obstacles) {
    
    // Simple collision check: sample points along the line
    int numSamples = 20;
    for (int i = 0; i <= numSamples; i++) {
        double t = static_cast<double>(i) / numSamples;
        Vertex sample(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y));
        
        if (pointInAnyObstacle(sample, obstacles)) {
            return true;
        }
    }
    return false;
}

double CombinatorialPlanning::distanceToNearestObstacle(
    const Vertex& point, const std::vector<Obstacle>& obstacles) {
    
    double minDist = std::numeric_limits<double>::max();
    
    for (const auto& obstacle : obstacles) {
        for (const auto& obstacle_point : obstacle.get_points()) {
            Vertex vertex = Vertex(point.x, point.y);
            double dist = point.distance(vertex);
            minDist = std::min(minDist, dist);
        }
    }
    
    return minDist;
}

// Helper to find where the vertical line at xMid intersects the map boundary
std::pair<double, double> CombinatorialPlanning::getMapYBoundsAtX(double x, const std::vector<Point>& boundary) {
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    bool found = false;

    size_t n = boundary.size();
    for (size_t i = 0; i < n; i++) {
        Point p1 = boundary[i];
        Point p2 = boundary[(i + 1) % n];

        // 1. Check if the vertical line at 'x' hits this edge segment
        // We use a small epsilon for vertex inclusion
        double minPx = std::min(p1.x, p2.x);
        double maxPx = std::max(p1.x, p2.x);

        if (x >= minPx - 1e-9 && x <= maxPx + 1e-9) {
            // Avoid division by zero for vertical edges
            if (std::abs(p2.x - p1.x) > 1e-9) {
                // Interpolate Y at X
                double t = (x - p1.x) / (p2.x - p1.x);
                double y = p1.y + t * (p2.y - p1.y);
                
                minY = std::min(minY, y);
                maxY = std::max(maxY, y);
                found = true;
            } else {
                // Vertical edge: take both endpoints
                minY = std::min(minY, std::min(p1.y, p2.y));
                maxY = std::max(maxY, std::max(p1.y, p2.y));
                found = true;
            }
        }
    }

    if (!found) {
        // Fallback for safety
        return {0.0, 0.0}; 
    }
    return {minY, maxY};
}