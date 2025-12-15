#include "roadmap/roadmap_visualization.h"
#include <algorithm>
#include <cmath>

namespace roadmap_viz {

RoadmapVisualizer::RoadmapVisualizer(const RoadmapVizConfig& config) 
    : config_(config) {
    canvas_ = cv::Mat(config_.img_height, config_.img_width, CV_8UC3, config_.color_background);
}

void RoadmapVisualizer::calculateBounds(const Map& map) {
    bounds_.points_found = false;
    
    // Inizializza con i bordi se presenti
    if (!map.borders.get_points().empty()) {
        bounds_.min_x = bounds_.max_x = static_cast<float>(map.borders.get_points()[0].x);
        bounds_.min_y = bounds_.max_y = static_cast<float>(map.borders.get_points()[0].y);
        bounds_.points_found = true;
    }
    
    auto update_bounds = [&](float x, float y) {
        if (!bounds_.points_found) {
            bounds_.min_x = bounds_.max_x = x;
            bounds_.min_y = bounds_.max_y = y;
            bounds_.points_found = true;
        } else {
            bounds_.min_x = std::min(bounds_.min_x, x);
            bounds_.max_x = std::max(bounds_.max_x, x);
            bounds_.min_y = std::min(bounds_.min_y, y);
            bounds_.max_y = std::max(bounds_.max_y, y);
        }
    };

    for (const auto& p : map.borders.get_points()) update_bounds(p.x, p.y);
    for (const auto& obs : map.obstacles.get_obstacles()) {
        for (const auto& p : obs.get_points()) update_bounds(p.x, p.y);
    }
    
    // Considera anche Vittime e Gate per il bounds (importante se sono fuori dai bordi per errore)
    for (const auto& v : map.victims.get_victims()) update_bounds(v.get_center().x, v.get_center().y);
    for (const auto& g : map.gates.get_gates()) update_bounds(g.get_position().x, g.get_position().y);

    if (!bounds_.points_found) {
        bounds_.min_x = bounds_.min_y = -10.0f;
        bounds_.max_x = bounds_.max_y = 10.0f;
    }
    
    // Calcolo scala con margini
    float world_width = bounds_.max_x - bounds_.min_x;
    float world_height = bounds_.max_y - bounds_.min_y;
    if (world_width <= 0) world_width = 1.0f;
    if (world_height <= 0) world_height = 1.0f;
    
    float avail_w = config_.img_width - 2 * config_.margin;
    float avail_h = config_.img_height - 2 * config_.margin;
    
    bounds_.scale = std::min(avail_w / world_width, avail_h / world_height);
}

cv::Point RoadmapVisualizer::worldToImage(float x, float y) const {
    int img_x = config_.margin + static_cast<int>((x - bounds_.min_x) * bounds_.scale);
    int img_y = config_.img_height - config_.margin - static_cast<int>((y - bounds_.min_y) * bounds_.scale);
    return cv::Point(img_x, img_y);
}

cv::Point RoadmapVisualizer::vertexToImage(const Vertex& v) const {
    return worldToImage(v.x, v.y);
}

float RoadmapVisualizer::quaternionToYaw(const Orientation& q) const {
    return std::atan2(2.0f * (q.w * q.z + q.x * q.y),
                     1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

// --- DISEGNO ELEMENTI MAPPA ---

void RoadmapVisualizer::drawBorders(const Borders& borders) {
    if (borders.get_points().size() < 2) return;
    std::vector<cv::Point> points;
    for (const auto& p : borders.get_points()) points.push_back(worldToImage(p.x, p.y));
    const cv::Point* pts = points.data();
    int npts = points.size();
    cv::polylines(canvas_, &pts, &npts, 1, true, config_.color_border, config_.border_thickness);
}

void RoadmapVisualizer::drawObstacles(const Obstacles& obstacles) {
    for (const auto& obs : obstacles.get_obstacles()) {
        if (obs.get_points().size() < 2) continue;
        std::vector<cv::Point> points;
        for (const auto& p : obs.get_points()) points.push_back(worldToImage(p.x, p.y));
        const cv::Point* pts = points.data();
        int npts = points.size();
        cv::fillPoly(canvas_, &pts, &npts, 1, config_.color_obstacle_fill);
        cv::polylines(canvas_, &pts, &npts, 1, true, config_.color_obstacle_outline, config_.obstacle_thickness);
    }
}

void RoadmapVisualizer::drawVictims(const Victims& victims) {
    for (const auto& v : victims.get_victims()) {
        cv::Point center = worldToImage(v.get_center().x, v.get_center().y);
        
        // Raggio approssimativo (clamped per visibilità)
        float r_meters = v.get_radius() / 1000.0f; // Assumendo raggio in mm nel msg originale
        int r_px = static_cast<int>(r_meters * bounds_.scale);
        int final_r = std::max(10, std::min(r_px, 100)); // Min 10px, Max 100px
        
        cv::circle(canvas_, center, final_r, config_.color_victim_fill, -1);
        cv::circle(canvas_, center, final_r, config_.color_victim_outline, 2);
        
        // Label "V"
        cv::putText(canvas_, "V", cv::Point(center.x - 5, center.y + 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 2);
    }
}

void RoadmapVisualizer::drawGates(const Gates& gates) {
    for (const auto& gate : gates.get_gates()) {
        cv::Point pos = worldToImage(gate.get_position().x, gate.get_position().y);
        float yaw = quaternionToYaw(gate.get_orientation());
        
        // Calcola fine della freccia (nota: asse Y immagine è invertito rispetto al mondo, quindi -sin)
        cv::Point arrow_end(
            pos.x + static_cast<int>(config_.gate_arrow_length * std::cos(yaw)),
            pos.y - static_cast<int>(config_.gate_arrow_length * std::sin(yaw))
        );
        
        // Disegna freccia e punto base
        cv::arrowedLine(canvas_, pos, arrow_end, config_.color_gate, 3, cv::LINE_AA, 0, 0.3);
        cv::circle(canvas_, pos, 6, config_.color_gate, -1);
        
        // Label "GATE"
        cv::putText(canvas_, "GATE", cv::Point(pos.x + 10, pos.y), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,100,0), 1);
    }
}

// --- DISEGNO ROADMAP ---

void RoadmapVisualizer::drawTrapezoids(const Roadmap& roadmap) {
    if (!roadmap.debugTrapezoids) return;
    cv::Scalar col(220, 220, 220); // Very Light Gray
    for (const auto& t : *roadmap.debugTrapezoids) {
        std::vector<cv::Point> pts = {
            worldToImage(t.leftX, t.topLeftY), worldToImage(t.rightX, t.topRightY),
            worldToImage(t.rightX, t.bottomRightY), worldToImage(t.leftX, t.bottomLeftY)
        };
        const cv::Point* points = pts.data();
        int npts = 4;
        cv::polylines(canvas_, &points, &npts, 1, true, col, 1);
    }
}

void RoadmapVisualizer::drawCells(const Roadmap& roadmap) {
    if (!roadmap.debugCells) return;
    cv::Scalar col(220, 220, 220);
    for (const auto& c : *roadmap.debugCells) {
        cv::rectangle(canvas_, worldToImage(c.minX, c.maxY), worldToImage(c.maxX, c.minY), col, 1);
    }
}

void RoadmapVisualizer::drawRoadmapEdges(const Roadmap& roadmap) {
    for (int i = 0; i < roadmap.getNumVertices(); i++) {
        cv::Point p1 = vertexToImage(roadmap.getVertex(i));
        for (const auto& edge : roadmap.getEdges(i)) {
            if (edge.targetVertex > i) { // Evita duplicati
                cv::Point p2 = vertexToImage(roadmap.getVertex(edge.targetVertex));
                cv::line(canvas_, p1, p2, config_.color_edge, config_.edge_thickness);
            }
        }
    }
}

void RoadmapVisualizer::drawRoadmapVertices(const Roadmap& roadmap) {
    for (int i = 0; i < roadmap.getNumVertices(); i++) {
        cv::Point p = vertexToImage(roadmap.getVertex(i));
        cv::circle(canvas_, p, config_.vertex_radius, config_.color_vertex, -1);
        cv::circle(canvas_, p, config_.vertex_radius, cv::Scalar(0,0,0), 1);
    }
}

void RoadmapVisualizer::drawInfo(const Roadmap& roadmap) {
    std::string info = "Nodes: " + std::to_string(roadmap.getNumVertices());
    cv::putText(canvas_, info, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,0), 2);
}

// --- RENDER & PATH ---

void RoadmapVisualizer::render(const Map& map, const Roadmap& roadmap) {
    canvas_ = cv::Mat(config_.img_height, config_.img_width, CV_8UC3, config_.color_background);
    calculateBounds(map);
    
    // Layer 1: Mappa Base
    drawBorders(map.borders);
    drawObstacles(map.obstacles);
    drawVictims(map.victims);  // <--- Vittime
    drawGates(map.gates);      // <--- Gate
    
    // Layer 2: Debug (Trapezi/Celle)
    drawTrapezoids(roadmap);
    drawCells(roadmap);
    
    // Layer 3: Grafo
    drawRoadmapEdges(roadmap);
    drawRoadmapVertices(roadmap);
    
    drawInfo(roadmap);
}

void RoadmapVisualizer::drawPath(const Roadmap& roadmap, const std::vector<int>& path) {
    if (path.size() < 2) return;
    
    cv::Scalar pathColor(0, 0, 255); // Rosso
    int thickness = 4;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        int u = path[i];
        int v = path[i+1];
        if (u < 0 || u >= roadmap.getNumVertices() || v < 0 || v >= roadmap.getNumVertices()) continue;
        
        cv::Point p1 = vertexToImage(roadmap.getVertex(u));
        cv::Point p2 = vertexToImage(roadmap.getVertex(v));
        
        cv::line(canvas_, p1, p2, pathColor, thickness, cv::LINE_AA);
        cv::circle(canvas_, p1, 4, pathColor, -1); // Waypoint
    }
    // Ultimo punto
    if (!path.empty()) {
        cv::circle(canvas_, vertexToImage(roadmap.getVertex(path.back())), 6, pathColor, -1);
    }
}

void RoadmapVisualizer::display() {
    cv::namedWindow(config_.window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(config_.window_name, 800, 600);
    cv::imshow(config_.window_name, canvas_);
    cv::waitKey(500);
}

bool RoadmapVisualizer::saveToFile(const std::string& filename) {
    return cv::imwrite(filename, canvas_);
}

} // namespace roadmap_viz