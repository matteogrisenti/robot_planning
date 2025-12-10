/* This file implement the library for the visualization of one roadmap. It is separate from the data strcutre to have a cleaner files and a
better organization */

#include <cmath>
#include <limits>

#include "map/map_visualization.h"

namespace map_viz {

// ============== MapVisualizer Implementation ==============

MapVisualizer::MapVisualizer(const VizConfig& config) 
    : config_(config) 
{
    canvas_ = cv::Mat(config_.img_height, config_.img_width, CV_8UC3, config_.color_background);
}

void MapVisualizer::calculateBounds(const Map& map) {
    bounds_.min_x = std::numeric_limits<float>::max();
    bounds_.max_x = std::numeric_limits<float>::lowest();
    bounds_.min_y = std::numeric_limits<float>::max();
    bounds_.max_y = std::numeric_limits<float>::lowest();
    bounds_.points_found = false;
    
    auto update_bounds = [&](float x, float y) {
        bounds_.min_x = std::min(bounds_.min_x, x);
        bounds_.max_x = std::max(bounds_.max_x, x);
        bounds_.min_y = std::min(bounds_.min_y, y);
        bounds_.max_y = std::max(bounds_.max_y, y);
        bounds_.points_found = true;
    };
    
    // Gather bounds from all sources
    for (const auto& p : map.borders.get_points()) {
        update_bounds(static_cast<float>(p.x), static_cast<float>(p.y));
    }
    
    for (const auto& obs : map.obstacles.get_obstacles()) {
        for (const auto& p : obs.get_points()) {
            update_bounds(static_cast<float>(p.x), static_cast<float>(p.y));
        }
    }
    
    for (const auto& v : map.victims.get_victims()) {
        update_bounds(static_cast<float>(v.get_center().x), 
                     static_cast<float>(v.get_center().y));
    }
    
    for (const auto& g : map.gates.get_gates()) {
        update_bounds(static_cast<float>(g.get_position().x), 
                     static_cast<float>(g.get_position().y));
    }
    
    if (map.start.get_position().x != 0 || map.start.get_position().y != 0) {
        update_bounds(static_cast<float>(map.start.get_position().x),
                    static_cast<float>(map.start.get_position().y));
    }
    
    // Handle empty map
    if (!bounds_.points_found) {
        ROS_WARN("[Visualizer] No data points found, using default bounds");
        bounds_.min_x = -5.0f; bounds_.max_x = 5.0f;
        bounds_.min_y = -5.0f; bounds_.max_y = 5.0f;
    }
    
    // Calculate world dimensions
    float world_width = bounds_.max_x - bounds_.min_x;
    float world_height = bounds_.max_y - bounds_.min_y;
    
    if (world_width < 0.1f) world_width = 10.0f;
    if (world_height < 0.1f) world_height = 10.0f;
    
    // Calculate scale
    float scale_x = (config_.img_width - 2.0f * config_.margin) / world_width;
    float scale_y = (config_.img_height - 2.0f * config_.margin) / world_height;
    bounds_.scale = std::min(scale_x, scale_y);
    
    ROS_INFO("[Visualizer] Bounds: x[%.2f, %.2f] y[%.2f, %.2f] | Scale: %.2f px/m", 
             bounds_.min_x, bounds_.max_x, bounds_.min_y, bounds_.max_y, bounds_.scale);
}

cv::Point MapVisualizer::worldToImage(float x, float y) const {
    int img_x = static_cast<int>((x - bounds_.min_x) * bounds_.scale + config_.margin);
    int img_y = static_cast<int>(config_.img_height - 
                                 ((y - bounds_.min_y) * bounds_.scale + config_.margin));
    return cv::Point(img_x, img_y);
}

float MapVisualizer::quaternionToYaw(const Orientation& q) const {
    return std::atan2(2.0f * (q.w * q.z + q.x * q.y),
                     1.0f - 2.0f * (q.y * q.y + q.z * q.z));
}

void MapVisualizer::drawBorders(const Borders& borders) {
    const auto& points = borders.get_points();
    if (points.size() < 2) return;
    
    std::vector<cv::Point> border_cv_points;
    for (const auto& p : points) {
        border_cv_points.push_back(worldToImage(
            static_cast<float>(p.x), static_cast<float>(p.y)));
    }
    
    const cv::Point* pts = border_cv_points.data();
    int npts = static_cast<int>(border_cv_points.size());
    cv::polylines(canvas_, &pts, &npts, 1, true, 
                  config_.color_border, config_.border_thickness);
}

void MapVisualizer::drawObstacles(const Obstacles& obstacles) {
    for (const auto& obstacle : obstacles.get_obstacles()) {
        const auto& vertices = obstacle.get_points();
        if (vertices.empty()) continue;
        
        std::vector<cv::Point> obs_cv_points;
        for (const auto& v : vertices) {
            obs_cv_points.push_back(worldToImage(
                static_cast<float>(v.x), static_cast<float>(v.y)));
        }
        
        // Draw polygon
        if (obs_cv_points.size() >= 3) {
            const cv::Point* pts = obs_cv_points.data();
            int npts = static_cast<int>(obs_cv_points.size());
            cv::fillPoly(canvas_, &pts, &npts, 1, config_.color_obstacle_fill);
            cv::polylines(canvas_, obs_cv_points, true, 
                         config_.color_obstacle_outline, config_.obstacle_thickness);
        }
        
        // Draw circle if has radius
        float radius = obstacle.get_radius();
        if (radius > 0.0f && !vertices.empty()) {
            cv::Point center = worldToImage(
                static_cast<float>(vertices[0].x), 
                static_cast<float>(vertices[0].y));
            int radius_px = static_cast<int>(radius * bounds_.scale);
            if (radius_px > 0) {
                cv::circle(canvas_, center, radius_px, config_.color_obstacle_fill, -1);
                cv::circle(canvas_, center, radius_px, config_.color_obstacle_outline, 2);
            }
        }
    }
}

void MapVisualizer::drawVictims(const Victims& victims) {
    for (const auto& victim : victims.get_victims()) {
        cv::Point center_px = worldToImage(
            static_cast<float>(victim.get_center().x),
            static_cast<float>(victim.get_center().y));
        
        float raw_radius = victim.get_radius();
        
        // Convert mm to meters (if needed)
        int radius_px = static_cast<int>((raw_radius / 1000.0f) * bounds_.scale);
        
        // Clamp radius to reasonable pixel size
        int final_radius = std::max(5, std::min(radius_px, 100));
        
        cv::circle(canvas_, center_px, final_radius, config_.color_victim_fill, -1);
        cv::circle(canvas_, center_px, final_radius, config_.color_victim_outline, 2);
    }
}

void MapVisualizer::drawGates(const Gates& gates) {
    for (const auto& gate : gates.get_gates()) {
        cv::Point gate_pos = worldToImage(
            static_cast<float>(gate.get_position().x),
            static_cast<float>(gate.get_position().y));
        
        float yaw = quaternionToYaw(gate.get_orientation());
        
        cv::Point arrow_end(
            gate_pos.x + static_cast<int>(config_.gate_arrow_length * std::cos(yaw)),
            gate_pos.y - static_cast<int>(config_.gate_arrow_length * std::sin(yaw))
        );
        
        cv::arrowedLine(canvas_, gate_pos, arrow_end, config_.color_gate, 3, cv::LINE_AA, 0, 0.3);
        cv::circle(canvas_, gate_pos, 5, config_.color_gate, -1);
    }
}

void MapVisualizer::drawStart(const Start& start) {
    const Point& position = start.get_position();
    const Orientation& orientation = start.get_orientation();
    
    if (position.x == 0 && position.y == 0) return;
    
    cv::Point start_pos = worldToImage(
        static_cast<float>(position.x),
        static_cast<float>(position.y));
    
    float yaw = quaternionToYaw(orientation);
    
    // Draw circle
    cv::circle(canvas_, start_pos, config_.start_circle_radius, 
               config_.color_start_fill, -1);
    cv::circle(canvas_, start_pos, config_.start_circle_radius, 
               config_.color_start_outline, 2);
    
    // Draw orientation arrow
    cv::Point arrow_end(
        start_pos.x + static_cast<int>(config_.start_arrow_length * std::cos(yaw)),
        start_pos.y - static_cast<int>(config_.start_arrow_length * std::sin(yaw))
    );
    cv::arrowedLine(canvas_, start_pos, arrow_end, 
                    cv::Scalar(255, 100, 0), 2, cv::LINE_AA, 0, 0.3);
    
    // Label
    cv::putText(canvas_, "START", cv::Point(start_pos.x + 15, start_pos.y - 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, config_.color_start_fill, 2);
}

void MapVisualizer::drawLegend() {
    int x = 20, y = 30;
    cv::putText(canvas_, 
                "Borders (Black) | Obstacles (Red) | Victims (Purple) | Gates (Green) | Start (Blue)", 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(50, 50, 50), 2);
}

void MapVisualizer::drawGrid() {
    if (!bounds_.points_found) return;
    cv::rectangle(canvas_, 
                  worldToImage(bounds_.min_x, bounds_.min_y), 
                  worldToImage(bounds_.max_x, bounds_.max_y), 
                  cv::Scalar(200, 200, 200), 1);
}

void MapVisualizer::render(const Map& map) {
    ROS_INFO("[Visualizer] Starting render...");
    
    // Reset canvas
    canvas_ = cv::Mat(config_.img_height, config_.img_width, CV_8UC3, config_.color_background);
    
    // Calculate bounds and scale
    calculateBounds(map);
    
    if (!bounds_.points_found) {
        cv::putText(canvas_, "NO DATA AVAILABLE", 
                    cv::Point(config_.img_width/2 - 200, config_.img_height/2), 
                    cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 0, 255), 3);
        return;
    }
    
    // Draw all elements in order (back to front)
    // drawGrid();  // Optional
    drawBorders(map.borders);
    drawObstacles(map.obstacles);
    drawVictims(map.victims);
    drawGates(map.gates);
    drawStart(map.start);
    drawLegend();
    
    ROS_INFO("[Visualizer] Render complete!");
}

void MapVisualizer::display() {
    try {
        cv::namedWindow(config_.window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(config_.window_name, 800, 600);
        cv::imshow(config_.window_name, canvas_);
        cv::waitKey(60);
    } catch (const cv::Exception& e) {
        ROS_WARN("[Visualizer] Display error: %s", e.what());
    }
}

bool MapVisualizer::saveToFile(const std::string& filename) {
    try {
        bool success = cv::imwrite(filename, canvas_);
        if (success) {
            ROS_INFO("[Visualizer] Saved to: %s", filename.c_str());
        } else {
            ROS_ERROR("[Visualizer] Failed to save to: %s", filename.c_str());
        }
        return success;
    } catch (const cv::Exception& e) {
        ROS_ERROR("[Visualizer] Save error: %s", e.what());
        return false;
    }
}

} // namespace roadmap_viz