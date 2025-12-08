// This file implement the main data structure for the managment of the roadmap creation

#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <ros/ros.h>
#include <limits>
#include <vector>
#include <algorithm>
#include <string>

#include "map/map_visualization.h"
#include "map/map_data_structures.h"


// ============== Borders ==============
Borders::Borders() = default;

const std::vector<Point>& Borders::get_points() const {
    return points;
}

void Borders::add_point(double x, double y, double z) {
    points.push_back({x, y, z});
}

void Borders::clear() {
    points.clear();
}

std::string Borders::to_string() const {
    std::ostringstream oss;
    oss << "Borders: " << points.size() << " vertices\n";
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        oss << "  Point " << i 
            << ": x=" << std::fixed << std::setprecision(3) << p.x
            << ", y=" << p.y 
            << ", z=" << p.z << "\n";
    }
    return oss.str();
}




// ============== Start ==============
Start::Start() : position({0, 0, 0}), orientation({0, 0, 0, 1}) {}
Start::Start(const Point& position, const Orientation& orientation)
    : position(position), orientation(orientation)
{
}

// Getter
const Point& Start::get_position() const { return position; }
const Orientation& Start::get_orientation() const { return orientation; }

std::string Start::to_string() const {
    std::stringstream oss;
    oss << "Start(position=(" 
        << std::fixed << std::setprecision(3) 
        << "x=" << position.x << ", y=" << position.y << ", z=" << position.z 
        << "), orientation=" 
        << std::fixed << std::setprecision(3)  
        << "x=" << orientation.x << ", y=" << orientation.y << ", z=" << orientation.z << ", w=" << orientation.w
        << ")";
    return oss.str();
}




// ============== Gate ==============
Gate::Gate(const Point& position, const Orientation& orientation)
    : position(position), orientation(orientation)
{
}

const Point& Gate::get_position() const {return position; }
const Orientation& Gate::get_orientation() const { return orientation; }

std::string Gate::to_string() const {
    std::stringstream oss;
    oss << "Gate(position=(" 
        << std::fixed << std::setprecision(3) 
        << "x=" << position.x << ", y=" << position.y << ", z=" << position.z 
        << "), orientation=" 
        << std::fixed << std::setprecision(3)  
        << "x=" << orientation.x << ", y=" << orientation.y << ", z=" << orientation.z << ", w=" << orientation.w
        << ")";
    return oss.str();
}




// ============== Gates ==============
Gates::Gates() = default;

const std::vector<Gate>& Gates::get_gates() const {
    return gates;
}

bool Gates::add_gate(const Point& position, const Orientation& orientation) {
    // Create and add the gate
    gates.emplace_back(position, orientation);
    
    return true;
}

void Gates::clear() {
    gates.clear();
}

std::string Gates::to_string() const {
    std::stringstream oss;
    oss << "Gates(count=" << gates.size() << ", gates=[\n";
    
    for (size_t i = 0; i < gates.size(); ++i) {
        oss << "  [" << i << "] " << gates[i].to_string();
        if (i < gates.size() - 1) {
            oss << ",\n";
        } else {
            oss << "\n";
        }
    }
    
    oss << "])";
    return oss.str();
}; 




// ============== Obstacle ==============
Obstacle::Obstacle(const std::vector<Point>* points_ptr, const float radius)
    : radius(radius)
{
    if (points_ptr != nullptr) {
        // Copy the values from the pointed vector
        points = *points_ptr;
    }
    // If points_ptr is null, points remains empty (default constructed)
}

const std::vector<Point>& Obstacle::get_points() const { return points; };
const float Obstacle::get_radius() const { return radius; };

// Get axis-aligned bounding box
void Obstacle::get_bounding_box(double& minX, double& minY, double& maxX, double& maxY) const {
    if (points.empty()) return;
    minX = maxX = points[0].x;
    minY = maxY = points[0].y;
    for (const auto& v : points) {
        minX = std::min(minX, v.x);
        maxX = std::max(maxX, v.x);
        minY = std::min(minY, v.y);
        maxY = std::max(maxY, v.y);
    }
}

std::string Obstacle::to_string() const {
    std::stringstream oss;
    oss << "Obstacle(radius=" << radius << ", vertices=[\n";
    
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& p = points[i];
        oss << "\t Point " << i 
            << ": x=" << std::fixed << std::setprecision(3) << p.x
            << ", y=" << p.y 
            << ", z=" << p.z;
        if (i < points.size() - 1) {
            oss << "\n";
        }
    }
    
    oss << "])";
    return oss.str();
};




// ============== Obstacles ==============
Obstacles::Obstacles() {
    // Default constructor - obstacles vector is already default-initialized as empty
}

const std::vector<Obstacle>& Obstacles::get_obstacles() const {
    return obstacles;
}

bool Obstacles::add_obstacle(const std::vector<Point>* points_ptr, const float radius) {
    // Check if pointer is valid
    if (points_ptr == nullptr) {
        return false;
    }
    
    // Check if points vector is empty (optional validation)
    if (points_ptr->empty()) {
        return false;
    }
    
    // Create and add the obstacle
    obstacles.emplace_back(points_ptr, radius);
    
    return true;
}

void Obstacles::clear() {
    obstacles.clear();
}

std::string Obstacles::to_string() const {
    std::stringstream ss;
    ss << "Obstacles(count=" << obstacles.size() << ", obstacles=[\n";
    
    for (size_t i = 0; i < obstacles.size(); ++i) {
        ss << "  [" << i << "] " << obstacles[i].to_string();
        if (i < obstacles.size() - 1) {
            ss << ",\n";
        } else {
            ss << "\n";
        }
    }
    
    ss << "])";
    return ss.str();
}




// ============== Victim ==============
Victim::Victim(const Point& center, const float radius)
    : center(center), radius(radius)
{
}

const Point& Victim::get_center() const {
    return center;
}

const float Victim::get_radius() const {
    return radius;
}

std::string Victim::to_string() const {
    std::stringstream oss;
    oss << "Victim(center=(" 
        << std::fixed << std::setprecision(3) 
        << center.x << ", " << center.y << ", " << center.z 
        << "), radius=" << radius << ")";
    return oss.str();
}




// ============== Victims ==============
Victims::Victims()  = default;

const std::vector<Victim>& Victims::get_victims() const {
    return victims;
}

bool Victims::add_victim(const Point& center, const float radius) {
    // Validate radius
    if (radius <= 0.0f) {
        return false;
    }
    
    // Create and add the victim
    victims.emplace_back(center, radius);
    
    return true;
}

void Victims::clear() {
    victims.clear();
}

std::string Victims::to_string() const {
    std::stringstream oss;
    oss << "Victims(count=" << victims.size() << ", victims=[\n";
    
    for (size_t i = 0; i < victims.size(); ++i) {
        oss << "  [" << i << "] " << victims[i].to_string();
        if (i < victims.size() - 1) {
            oss << ",\n";
        } else {
            oss << "\n";
        }
    }
    
    oss << "])";
    return oss.str();
}




// ============== Map ==============
Map::Map() = default;

void Map::paint_map() {
    // Create visualizer with default config
    map_viz::MapVisualizer visualizer;
    
    // Render the roadmap
    visualizer.render(*this);
    
    // Display
    visualizer.display();
    
    // Optionally save to file
    // visualizer.saveToFile("/tmp/roadmap.png");
}

// Get map bounding box
void  Map::get_bounding_box(double& minX, double& minY, double& maxX, double& maxY) const {
    std::vector<Point> border_verteces = borders.get_points();
    if (border_verteces.empty()) return;
    minX = maxX = border_verteces[0].x;
    minY = maxY = border_verteces[0].y;
    for (const auto& v : border_verteces) {
        minX = std::min(minX, v.x);
        maxX = std::max(maxX, v.x);
        minY = std::min(minY, v.y);
        maxY = std::max(maxY, v.y);
    }
}