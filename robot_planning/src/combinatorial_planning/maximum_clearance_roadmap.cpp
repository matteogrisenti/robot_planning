#include "combinatorial_planning/maximum_clearance_roadmap.h"

#include <vector>
#include <map>
#include <cmath>
#include <iostream>

// Boost.Polygon headers
#include <boost/polygon/voronoi.hpp>
/* Too complex to implement by scratch; we use an external library which work*/

// OpenCV for geometric checks
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// =========================================================
// INTERNAL HELPER STRUCTURES
// =========================================================

const double SCALING_FACTOR = 1000.0;

struct PointInt {
    int x, y;
};

struct SegmentInt {
    PointInt p0;
    PointInt p1;
};

// =========================================================
// BOOST.POLYGON TRAITS SPECIALIZATION
// =========================================================
namespace boost {
namespace polygon {

// --- FIX: Added semicolon at the end ---
template <>
struct geometry_concept<PointInt> { typedef point_concept type; };

template <>
struct point_traits<PointInt> {
    typedef int coordinate_type;
    static inline coordinate_type get(const PointInt& point, orientation_2d orient) {
        return (orient == HORIZONTAL) ? point.x : point.y;
    }
};

// --- FIX: Added semicolon at the end ---
template <>
struct geometry_concept<SegmentInt> { typedef segment_concept type; };

template <>
struct segment_traits<SegmentInt> {
    typedef int coordinate_type;
    typedef PointInt point_type;
    static inline point_type get(const SegmentInt& segment, direction_1d dir) {
        return dir.to_int() ? segment.p1 : segment.p0;
    }
};

} // namespace polygon
} // namespace boost

// =========================================================
// PRIVATE HELPER FUNCTIONS
// =========================================================

void addPolygonToSegments(const std::vector<Point>& polyPoints, std::vector<SegmentInt>& segments) {
    if (polyPoints.size() < 2) return;
    for (size_t i = 0; i < polyPoints.size(); ++i) {
        size_t next = (i + 1) % polyPoints.size();
        SegmentInt s;
        s.p0.x = static_cast<int>(polyPoints[i].x * SCALING_FACTOR);
        s.p0.y = static_cast<int>(polyPoints[i].y * SCALING_FACTOR);
        s.p1.x = static_cast<int>(polyPoints[next].x * SCALING_FACTOR);
        s.p1.y = static_cast<int>(polyPoints[next].y * SCALING_FACTOR);
        segments.push_back(s);
    }
}

// Check if a point is navigable (inside map borders, outside obstacles)
bool isPointValid(double x, double y, const Map& map) {
    cv::Point2f pt((float)x, (float)y);

    // 1. Must be INSIDE the map borders
    std::vector<cv::Point2f> borderPts;
    for (const auto& p : map.borders.get_points()) {
        borderPts.emplace_back((float)p.x, (float)p.y);
    }
    // pointPolygonTest returns > 0 if inside, < 0 if outside
    if (cv::pointPolygonTest(borderPts, pt, false) < 0) {
        return false; 
    }

    // 2. Must be OUTSIDE all obstacles
    for (const auto& obs : map.obstacles.get_obstacles()) {
        std::vector<cv::Point2f> obsPts;
        for (const auto& p : obs.get_points()) {
            obsPts.emplace_back((float)p.x, (float)p.y);
        }
        // pointPolygonTest returns >= 0 if inside or on edge
        if (cv::pointPolygonTest(obsPts, pt, false) >= 0) {
            return false; 
        }
    }

    return true;
}

// =========================================================
// PUBLIC FUNCTION IMPLEMENTATION
// =========================================================

Roadmap generateMaxClearanceRoadmap(const Map& map) {
    Roadmap roadmap;
    roadmap.setMap(&map); 

    // 1. Prepare Input Segments
    std::vector<SegmentInt> segments;
    addPolygonToSegments(map.borders.get_points(), segments);
    for (const auto& obs : map.obstacles.get_obstacles()) {
        addPolygonToSegments(obs.get_points(), segments);
    }

    // 2. Run Boost Voronoi
    boost::polygon::voronoi_diagram<double> vd;
    boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vd);

    // 3. Convert to Roadmap with Filtering
    std::map<const boost::polygon::voronoi_diagram<double>::vertex_type*, int> vertex_lookup;

    auto get_roadmap_idx = [&](const boost::polygon::voronoi_diagram<double>::vertex_type* v) {
        if (vertex_lookup.find(v) == vertex_lookup.end()) {
            double rx = v->x() / SCALING_FACTOR;
            double ry = v->y() / SCALING_FACTOR;
            int idx = roadmap.addVertex(Vertex(rx, ry));
            vertex_lookup[v] = idx;
        }
        return vertex_lookup[v];
    };

    for (const auto& edge : vd.edges()) {
        if (!edge.is_primary()) continue; 
        if (edge.is_infinite()) continue; 

        const auto* v0 = edge.vertex0();
        const auto* v1 = edge.vertex1();

        if (v0 && v1) {
            double x0 = v0->x() / SCALING_FACTOR;
            double y0 = v0->y() / SCALING_FACTOR;
            double x1 = v1->x() / SCALING_FACTOR;
            double y1 = v1->y() / SCALING_FACTOR;

            // Check if both ends are valid
            if (isPointValid(x0, y0, map) && isPointValid(x1, y1, map)) {
                int id0 = get_roadmap_idx(v0);
                int id1 = get_roadmap_idx(v1);
                roadmap.addEdge(id0, id1, true);
            }
        }
    }

    return roadmap;
}