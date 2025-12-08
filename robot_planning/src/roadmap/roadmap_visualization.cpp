// =============================================================================
// IMPLEMENTATION FILE (roadmap_visualization.cpp)
// =============================================================================

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
    
    // Initialize with first border point if available
    if (!map.borders.get_points().empty()) {
        bounds_.min_x = bounds_.max_x = static_cast<float>(map.borders.get_points()[0].x);
        bounds_.min_y = bounds_.max_y = static_cast<float>(map.borders.get_points()[0].y);
        bounds_.points_found = true;
    }
    
    // Process all border points
    for (const auto& point : map.borders.get_points()) {
        bounds_.min_x = std::min(bounds_.min_x, static_cast<float>(point.x));
        bounds_.max_x = std::max(bounds_.max_x, static_cast<float>(point.x));
        bounds_.min_y = std::min(bounds_.min_y, static_cast<float>(point.y));
        bounds_.max_y = std::max(bounds_.max_y, static_cast<float>(point.y));
    }
    
    // Process all obstacles
    for (const auto& obstacle : map.obstacles.get_obstacles()) {
        for (const auto& point : obstacle.get_points()) {
            float px = static_cast<float>(point.x);
            float py = static_cast<float>(point.y);

            if (!bounds_.points_found) {
                bounds_.min_x = bounds_.max_x = px;
                bounds_.min_y = bounds_.max_y = py;
                bounds_.points_found = true;
            }
            
            bounds_.min_x = std::min(bounds_.min_x, px);
            bounds_.max_x = std::max(bounds_.max_x, px);
            bounds_.min_y = std::min(bounds_.min_y, py);
            bounds_.max_y = std::max(bounds_.max_y, py);
        }
    }
    
    if (!bounds_.points_found) {
        ROS_WARN("No points found in map for bounds calculation");
        bounds_.min_x = bounds_.min_y = 0.0f;
        bounds_.max_x = bounds_.max_y = 1.0f;
    }
    
    // Calculate scale to fit in canvas with margins
    float world_width = bounds_.max_x - bounds_.min_x;
    float world_height = bounds_.max_y - bounds_.min_y;
    
    if (world_width <= 0) world_width = 1.0f;
    if (world_height <= 0) world_height = 1.0f;
    
    float available_width = config_.img_width - 2 * config_.margin;
    float available_height = config_.img_height - 2 * config_.margin;
    
    float scale_x = available_width / world_width;
    float scale_y = available_height / world_height;
    
    bounds_.scale = std::min(scale_x, scale_y);
}

cv::Point RoadmapVisualizer::worldToImage(float x, float y) const {
    int img_x = config_.margin + static_cast<int>((x - bounds_.min_x) * bounds_.scale);
    int img_y = config_.img_height - config_.margin - 
                static_cast<int>((y - bounds_.min_y) * bounds_.scale);
    return cv::Point(img_x, img_y);
}

cv::Point RoadmapVisualizer::vertexToImage(const Vertex& v) const {
    return worldToImage(v.x, v.y);
}

void RoadmapVisualizer::drawBorders(const Borders& borders) {
    if (borders.get_points().size() < 2) return;
    
    std::vector<cv::Point> points;
    for (const auto& point : borders.get_points()) {
        points.push_back(worldToImage(point.x, point.y));
    }
    
    // Draw border polygon
    const cv::Point* pts = points.data();
    int npts = points.size();
    cv::polylines(canvas_, &pts, &npts, 1, true, 
                  config_.color_border, config_.border_thickness);
}

void RoadmapVisualizer::drawObstacles(const Obstacles& obstacles) {
    for (const auto& obstacle : obstacles.get_obstacles()) {
        if (obstacle.get_points().size() < 2) continue;
        
        std::vector<cv::Point> points;
        for (const auto& point : obstacle.get_points()) {
            points.push_back(worldToImage(point.x, point.y));
        }
        
        // Fill obstacle
        const cv::Point* pts = points.data();
        int npts = points.size();
        cv::fillPoly(canvas_, &pts, &npts, 1, config_.color_obstacle_fill);
        
        // Draw obstacle outline
        cv::polylines(canvas_, &pts, &npts, 1, true, 
                     config_.color_obstacle_outline, config_.obstacle_thickness);
    }
}

void RoadmapVisualizer::drawRoadmapEdges(const std::shared_ptr<Roadmap>& roadmap) {
    if (!roadmap) return;
    
    int numVertices = roadmap->getNumVertices();
    
    // Draw all edges
    for (int i = 0; i < numVertices; i++) {
        const Vertex& v1 = roadmap->getVertex(i);
        cv::Point p1 = vertexToImage(v1);
        
        const std::vector<Edge>& edges = roadmap->getEdges(i);
        for (const auto& edge : edges) {
            // Only draw each edge once (check if target index > current to avoid duplicates)
            if (edge.targetVertex > i) {
                const Vertex& v2 = roadmap->getVertex(edge.targetVertex);
                cv::Point p2 = vertexToImage(v2);
                
                // Draw edge line in electric blue
                cv::line(canvas_, p1, p2, config_.color_edge, config_.edge_thickness);
            }
        }
    }
}

void RoadmapVisualizer::drawRoadmapVertices(const std::shared_ptr<Roadmap>& roadmap) {
    if (!roadmap) return;
    
    int numVertices = roadmap->getNumVertices();
    
    // Draw all vertices as filled circles
    for (int i = 0; i < numVertices; i++) {
        const Vertex& v = roadmap->getVertex(i);
        cv::Point p = vertexToImage(v);
        
        // Draw big dot in electric blue
        cv::circle(canvas_, p, config_.vertex_radius, config_.color_vertex, -1); // -1 = filled
        
        // Optional: Draw small black outline for better visibility
        cv::circle(canvas_, p, config_.vertex_radius, cv::Scalar(0, 0, 0), 1);
    }
}

void RoadmapVisualizer::drawTrapezoids(const std::shared_ptr<Roadmap>& roadmap) {
    if (!roadmap || !roadmap->debugTrapezoids || roadmap->debugTrapezoids->empty()) return;

    cv::Scalar gridColor(211, 211, 211); // Light Gray

    for (const auto& trap : *roadmap->debugTrapezoids) {
        // Use the new 4 corners
        cv::Point p1 = worldToImage(trap.leftX, trap.topLeftY);     // Top-Left
        cv::Point p2 = worldToImage(trap.rightX, trap.topRightY);    // Top-Right
        cv::Point p3 = worldToImage(trap.rightX, trap.bottomRightY); // Bottom-Right
        cv::Point p4 = worldToImage(trap.leftX, trap.bottomLeftY);   // Bottom-Left

        std::vector<cv::Point> pts = {p1, p2, p3, p4};
        const cv::Point* points = pts.data();
        int npts = pts.size();

        cv::polylines(canvas_, &points, &npts, 1, true, gridColor, 1, cv::LINE_AA);
    }
}

void RoadmapVisualizer::drawCells(const std::shared_ptr<Roadmap>& roadmap) {
    // 1. Check if the pointer exists
    if (!roadmap || !roadmap->debugCells || roadmap->debugCells->empty()) {
        return;
    }

    // Light gray color
    cv::Scalar gridColor(211, 211, 211);

    for (const auto& cell : *roadmap->debugCells) {
        // Cells are Axis-Aligned Bounding Boxes (AABB) defined by min/max X/Y
        cv::Point topLeft = worldToImage(cell.minX, cell.maxY);
        cv::Point bottomRight = worldToImage(cell.maxX, cell.minY);

        // Draw rectangle
        cv::rectangle(canvas_, topLeft, bottomRight, gridColor, 1, cv::LINE_AA);
    }
}

void RoadmapVisualizer::drawInfo(const std::shared_ptr<Roadmap>& roadmap) {
    if (!roadmap) return;
    
    // Draw statistics in top-left corner
    int numVertices = roadmap->getNumVertices();
    int totalEdges = 0;
    for (int i = 0; i < numVertices; i++) {
        totalEdges += roadmap->getEdges(i).size();
    }
    totalEdges /= 2; // Bidirectional edges counted twice
    
    std::string info1 = "Vertices: " + std::to_string(numVertices);
    std::string info2 = "Edges: " + std::to_string(totalEdges);
    
    int baseline = 0;
    cv::Size textSize1 = cv::getTextSize(info1, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
    
    // Draw black background for text
    cv::rectangle(canvas_, 
                  cv::Point(10, 10),
                  cv::Point(textSize1.width + 30, 70),
                  cv::Scalar(0, 0, 0),
                  -1);
    
    // Draw white text
    cv::putText(canvas_, info1, cv::Point(20, 35), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    cv::putText(canvas_, info2, cv::Point(20, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
}

void RoadmapVisualizer::render(const Map& map, const std::shared_ptr<Roadmap>& roadmap) {
    // Reset canvas
    canvas_ = cv::Mat(config_.img_height, config_.img_width, CV_8UC3, config_.color_background);
    
    // Calculate world bounds
    calculateBounds(map);
    
    // 1. Draw Map Base
    drawBorders(map.borders);
    drawObstacles(map.obstacles);

    // 2. Draw Decomposition (Debug Layer)
    drawTrapezoids(roadmap);
    drawCells(roadmap);
    
    // 3. Draw Roadmap Graph (Foreground)
    drawRoadmapEdges(roadmap);
    drawRoadmapVertices(roadmap);
    
    // 4. Draw Info
    drawInfo(roadmap);
    
    ROS_INFO("Roadmap visualization rendered successfully");
}

void RoadmapVisualizer::display() {
    cv::namedWindow(config_.window_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(config_.window_name, 800, 600);
    cv::imshow(config_.window_name, canvas_);
    
    // FIX: Wait for the window to fully initialize before checking properties.
    // Without this, getWindowProperty often returns 0 immediately on Linux/ROS.
    cv::waitKey(500); 

    ROS_INFO("Displaying roadmap visualization. Press any key or click 'X' to continue...");
    
    while (true) {
        // Check if window was closed via X button
        // WND_PROP_VISIBLE == 0 means the window is not visible (closed)
        double prop = cv::getWindowProperty(config_.window_name, cv::WND_PROP_VISIBLE);
        
        // If prop is 0, window is closed. 
        // We ensure prop is not -1 (which would mean property not supported/error)
        if (prop == 0) {
            ROS_INFO("Window closed by user.");
            break;
        }

        // Wait for 100ms for a key press
        int key = cv::waitKey(100);
        if (key != -1) {
            ROS_INFO("Key pressed by user.");
            break;
        }
    }
    
    cv::destroyWindow(config_.window_name);
}

bool RoadmapVisualizer::saveToFile(const std::string& filename) {
    bool success = cv::imwrite(filename, canvas_);
    if (success) {
        ROS_INFO("Roadmap visualization saved to: %s", filename.c_str());
    } else {
        ROS_ERROR("Failed to save roadmap visualization to: %s", filename.c_str());
    }
    return success;
}

} // namespace roadmap_viz