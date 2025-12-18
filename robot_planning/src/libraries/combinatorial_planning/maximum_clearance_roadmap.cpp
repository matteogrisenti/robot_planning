#include "libraries/combinatorial_planning/maximum_clearance_roadmap.h"
#include "planning_utils.h" 

#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include <memory>

// Boost.Polygon headers
#include <boost/polygon/voronoi.hpp>
/* Too complex to implement by scratch; we use an external library which work*/



// BOOST.POLYGON TRAITS SPECIALIZATION
namespace boost {
namespace polygon {

// --- FIX: Added semicolon at the end ---
template <>
struct geometry_concept<HelperMaxClearanceRoadmap::PointInt> { typedef point_concept type; };

template <>
struct point_traits<HelperMaxClearanceRoadmap::PointInt> {
    typedef int coordinate_type;
    static inline coordinate_type get(const HelperMaxClearanceRoadmap::PointInt& point, orientation_2d orient) {
        return (orient == HORIZONTAL) ? point.x : point.y;
    }
};

// --- FIX: Added semicolon at the end ---
template <>
struct geometry_concept<HelperMaxClearanceRoadmap::SegmentInt> { typedef segment_concept type; };

template <>
struct segment_traits<HelperMaxClearanceRoadmap::SegmentInt> {
    typedef int coordinate_type;
    typedef HelperMaxClearanceRoadmap::PointInt point_type;
    static inline point_type get(const HelperMaxClearanceRoadmap::SegmentInt& segment, direction_1d dir) {
        return dir.to_int() ? segment.p1 : segment.p0;
    }
};

} // namespace polygon
} // namespace boost





std::shared_ptr<Roadmap> maximumClearanceRoadmap(const Map& map) {
    std::shared_ptr<Roadmap> roadmap = std::make_shared<Roadmap>();
    roadmap->setMap(&map); 

    // 1. Prepare Input Segments
    std::vector<HelperMaxClearanceRoadmap::SegmentInt> segments;
    HelperMaxClearanceRoadmap::addPolygonToSegments(map.borders.get_points(), segments);
    for (const auto& obs : map.obstacles.get_obstacles()) {
        HelperMaxClearanceRoadmap::addPolygonToSegments(obs.get_points(), segments);
    }

    // 2. Run Boost Voronoi
    boost::polygon::voronoi_diagram<double> vd;
    boost::polygon::construct_voronoi(segments.begin(), segments.end(), &vd);

    // 3. Convert to Roadmap with Filtering
    std::map<const boost::polygon::voronoi_diagram<double>::vertex_type*, int> vertex_lookup;

    auto get_roadmap_idx = [&](const boost::polygon::voronoi_diagram<double>::vertex_type* v) {
        if (vertex_lookup.find(v) == vertex_lookup.end()) {
            double rx = v->x() / HelperMaxClearanceRoadmap::SCALING_FACTOR;
            double ry = v->y() / HelperMaxClearanceRoadmap::SCALING_FACTOR;
            int idx = roadmap->addVertex(Vertex(rx, ry));
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
            double x0 = v0->x() / HelperMaxClearanceRoadmap::SCALING_FACTOR;
            double y0 = v0->y() / HelperMaxClearanceRoadmap::SCALING_FACTOR;
            double x1 = v1->x() / HelperMaxClearanceRoadmap::SCALING_FACTOR;
            double y1 = v1->y() / HelperMaxClearanceRoadmap::SCALING_FACTOR;

            // Check if both ends are valid
            if (PlanningUtils::isPointValid(x0, y0, map) && PlanningUtils::isPointValid(x1, y1, map)) {
                int id0 = get_roadmap_idx(v0);
                int id1 = get_roadmap_idx(v1);
                roadmap->addEdge(id0, id1, true);
            }
        }
    }

    return roadmap;
}




namespace HelperMaxClearanceRoadmap {

    // Converts a polygon defined by Points into segments and adds them to the list
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


}

