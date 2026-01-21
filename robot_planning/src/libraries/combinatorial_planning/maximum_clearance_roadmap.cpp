#include "libraries/combinatorial_planning/maximum_clearance_roadmap.h"
#include "libraries/planning_utils.h" 

#include <vector>
#include <map>
#include <cmath>
#include <iostream>
#include <memory>
#include <limits>
#include <algorithm>
#include <boost/polygon/voronoi.hpp>

namespace boost {
namespace polygon {

template <>
struct geometry_concept<HelperMaxClearanceRoadmap::PointInt> { typedef point_concept type; };

template <>
struct point_traits<HelperMaxClearanceRoadmap::PointInt> {
    typedef int coordinate_type;
    static inline coordinate_type get(const HelperMaxClearanceRoadmap::PointInt& point, orientation_2d orient) {
        return (orient == HORIZONTAL) ? point.x : point.y;
    }
};

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

    // 3. Dense Conversion (Sampling)
    double sampling_density = 0.25; 

    // Helper function to find or create nodes (avoids nearby duplicates)
    auto get_or_create_node = [&](double x, double y) {
        for(int i=0; i<roadmap->getNumVertices(); ++i) {
            if (std::hypot(roadmap->getVertex(i).x - x, roadmap->getVertex(i).y - y) < 0.05) {
                return i;
            }
        }
        return roadmap->addVertex(Vertex(x, y));
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

            bool safeStart = PlanningUtils::isPointValid(x0, y0, map, 0.5);
            bool safeEnd   = PlanningUtils::isPointValid(x1, y1, map, 0.5);
            if (safeStart && safeEnd) {
                
                // Calculate length and sampling steps
                double dist = std::hypot(x1 - x0, y1 - y0);
                int steps = std::max(1, (int)(dist / sampling_density));
                
                int prev_node_idx = get_or_create_node(x0, y0);
                
                for(int i = 1; i <= steps; ++i) {
                    double t = (double)i / steps;
                    double x_curr = x0 + t * (x1 - x0);
                    double y_curr = y0 + t * (y1 - y0);
                    
                    int curr_node_idx = get_or_create_node(x_curr, y_curr);
                    roadmap->addEdge(prev_node_idx, curr_node_idx, true);
                    prev_node_idx = curr_node_idx;
                }
            }
        }
    }

    // 4. Attach targets to dense roadmap
    std::vector<Vertex> targets;
    if (!map.gates.get_gates().empty()) {
        Point g = map.gates.get_gates()[0].get_position();
        targets.push_back(Vertex(g.x, g.y));
    }
    for (const auto& v : map.victims.get_victims()) {
        Point p = v.get_center();
        targets.push_back(Vertex(p.x, p.y));
    }

    for (const auto& target : targets) {
        int targetNodeIdx = roadmap->addVertex(target);

        // Attach to K nearest nodes of the sampled roadmap
        std::vector<std::pair<double, int>> candidates;
        double search_radius = 5.0;

        for (int i = 0; i < targetNodeIdx; ++i) { // Iterate over existing nodes
            double d = roadmap->getVertex(i).distance(target);
            if (d < search_radius) {
                if (!PlanningUtils::lineSegmentIntersectsObstacle(roadmap->getVertex(i), target, map.obstacles.get_obstacles())) {
                    candidates.push_back({d, i});
                }
            }
        }

        // Sort and take the best 3 (or less if there are none)
        std::sort(candidates.begin(), candidates.end());
        int k_conn = 3;
        for (int k = 0; k < std::min((int)candidates.size(), k_conn); ++k) {
            roadmap->addEdge(targetNodeIdx, candidates[k].second, true);
        }
        
        if (candidates.empty()) {
             ROS_WARN("MCR: Target isolato a (%.2f, %.2f)", target.x, target.y);
        }
    }

    return roadmap;
}

namespace HelperMaxClearanceRoadmap {
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