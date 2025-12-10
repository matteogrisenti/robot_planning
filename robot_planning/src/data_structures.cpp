
/* 
This file have been complety update. It has been spleeted the concept of Map and Roadmap.
Map:    data that describe the map: borders, obstacle, victims, gates, robot start point
Roadmap:    a set of vertex and edges that describe a roadmap structure ( more simpler and theoreticaly corent )

All the logic of the Collision Checker has been moved inside the class CollisionChecker make it an indipendent unity

*/

/*
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <ros/ros.h>
#include <limits>
#include <vector>
#include <algorithm>
#include <string>

#include "roadmap/data_structures.h"

 
// --- Costruttore ---
Roadmap::Roadmap(double robot_radius, double safety_margin) 
    : collision_checker_(robot_radius, safety_margin) 
{
}

// --- INTEGRAZIONE: Conversione Dati (Baking) ---

// --- INTEGRAZIONE: Check Puntuale ---
bool Roadmap::is_state_valid(double x, double y) const {
    // Ritorna TRUE se il punto è LIBERO (CollisionChecker.check ritorna true se COLLIDE)
    return !collision_checker_.check({x, y}, cached_circles_, cached_polygons_);
}

// --- INTEGRAZIONE: Check Dubins Path ---
bool Roadmap::is_dubins_path_valid(const Point& start, double start_theta, 
                                   const Point& end, double end_theta, 
                                   double rho) const {
    
    // Configura parametri Dubins
    double k_max = 1.0 / rho;
    int best_word = -1;
    dubinscurve_out curve;

    // 1. Calcola la curva
    dubins_shortest_path(start.x, start.y, start_theta, 
                         end.x, end.y, end_theta, 
                         k_max, best_word, &curve);

    if (best_word < 0) return false; // Percorso impossibile

    // 2. Campiona e valida ogni punto lungo la curva
    double step_size = 0.1; // Check ogni 10cm
    
    auto check_arc = [&](const dubinsarc_out& arc) -> bool {
        int steps = std::ceil(arc.l / step_size);
        for (int i = 0; i <= steps; ++i) {
            double s = (i * step_size > arc.l) ? arc.l : i * step_size;
            long double x, y, th;
            circline(s, arc.x0, arc.y0, arc.th0, arc.k, x, y, th);

            // Se anche un solo punto collide, il percorso è invalido
            if (!is_state_valid((double)x, (double)y)) {
                return false; 
            }
        }
        return true;
    };

    if (!check_arc(curve.a1)) return false;
    if (!check_arc(curve.a2)) return false;
    if (!check_arc(curve.a3)) return false;

    return true; // Percorso pulito
}

// ============== MapBorders ==============
MapBorders::MapBorders() = default;

const std::vector<Point>& MapBorders::get_points() const {
    return points;
}

void MapBorders::add_point(double x, double y, double z) {
    points.push_back({x, y, z});
}

void MapBorders::clear() {
    points.clear();
}

std::string MapBorders::to_string() const {
    std::ostringstream oss;
    oss << "MapBorders: " << points.size() << " vertices\n";
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
Start::Start(const Point& position, const Orientation& orientation)
    : position(position), orientation(orientation)
{
}

const Point& Start::get_position() const {
    return position;
}

const Orientation& Start::get_orientation() const {
    return orientation;
}

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

const Point& Gate::get_position() const {
    return position;
}

const Orientation& Gate::get_orientation() const {
    return orientation;
}

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

const std::vector<Point>& Obstacle::get_points() const {
    return points;
};

const float Obstacle::get_radius() const {
    return radius;
};

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




// ============== Roadmap ==============
void Roadmap::paint_roadmap() {
    ROS_INFO("[Roadmap] Starting visualization...");

    // Canvas settings
    const int IMG_WIDTH = 1600;
    const int IMG_HEIGHT = 1600;
    const int MARGIN = 50;

    // 1. Create white background
    cv::Mat canvas(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));

    // 2. Initialize Bounds
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    bool points_found = false;

    // --- HELPER LAMBDA TO UPDATE BOUNDS ---
    auto update_bounds = [&](float x, float y) {
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        points_found = true;
    };

    // 3. Gather Bounds from ALL sources (not just borders)
    const auto& border_points = mapBorders.get_points();
    for (const auto& p : border_points) update_bounds(static_cast<float>(p.x), static_cast<float>(p.y));

    const auto& obstacle_list = obstacles.get_obstacles();
    for (const auto& obs : obstacle_list) {
        for (const auto& p : obs.get_points()) update_bounds(static_cast<float>(p.x), static_cast<float>(p.y));
    }

    const auto& victim_list = victims.get_victims();
    for (const auto& v : victim_list) update_bounds(static_cast<float>(v.get_center().x), static_cast<float>(v.get_center().y));
    
    const auto& gate_list = gates.get_gates();
    for (const auto& g : gate_list) update_bounds(static_cast<float>(g.get_position().x), static_cast<float>(g.get_position().y));

    // 4. Handle Empty Map / Scale Calculation
    if (!points_found) {
        ROS_WARN("[Roadmap] Map is empty! Using default bounds.");
        min_x = -5.0f; max_x = 5.0f;
        min_y = -5.0f; max_y = 5.0f;
        
        cv::putText(canvas, "NO DATA RECEIVED", cv::Point(IMG_WIDTH/2 - 200, IMG_HEIGHT/2), 
                    cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0, 0, 255), 3);
    }

    // Add padding to world bounds so objects aren't on the exact edge of the image
    float world_width = max_x - min_x;
    float world_height = max_y - min_y;

    // Safety for flat lines or single points
    if (world_width < 0.1f) world_width = 10.0f; 
    if (world_height < 0.1f) world_height = 10.0f;

    // Calculate Scale (preserve aspect ratio)
    float scale_x = (IMG_WIDTH - 2.0f * MARGIN) / world_width;
    float scale_y = (IMG_HEIGHT - 2.0f * MARGIN) / world_height;
    float scale = std::min(scale_x, scale_y);

    ROS_INFO("[Roadmap] Bounds: x[%.2f, %.2f] y[%.2f, %.2f] | Scale: %.2f px/m", min_x, max_x, min_y, max_y, scale);

    // Lambda: World -> Image
    auto worldToImage = [&](float x, float y) -> cv::Point {
        int img_x = static_cast<int>((x - min_x) * scale + MARGIN);
        // Flip Y because Image (0,0) is top-left, World (0,0) is usually center/bottom-left
        int img_y = static_cast<int>(IMG_HEIGHT - ((y - min_y) * scale + MARGIN)); 
        return cv::Point(img_x, img_y);
    };

    // 5. DRAWING OPERATIONS

    // Draw Grid (Optional: helps see scale)
    if (points_found) {
        cv::rectangle(canvas, worldToImage(min_x, min_y), worldToImage(max_x, max_y), cv::Scalar(200, 200, 200), 1);
    }

    // Draw Borders
    if (border_points.size() >= 2) {
        std::vector<cv::Point> border_cv_points;
        for (const auto& p : border_points) border_cv_points.push_back(worldToImage((float)p.x, (float)p.y));
        
        // Draw connected lines
        const cv::Point* pts = border_cv_points.data();
        int npts = (int)border_cv_points.size();
        cv::polylines(canvas, &pts, &npts, 1, true, cv::Scalar(0, 0, 0), 3);
    }

    // Draw Obstacles
    for (const auto& obstacle : obstacle_list) {
        std::vector<cv::Point> obs_cv_points;
        for (const auto& v : obstacle.get_points()) {
            obs_cv_points.push_back(worldToImage((float)v.x, (float)v.y));
        }

        if (obs_cv_points.size() >= 3) {
            const cv::Point* pts = obs_cv_points.data();
            int npts = (int)obs_cv_points.size();
            cv::fillPoly(canvas, &pts, &npts, 1, cv::Scalar(0, 0, 255)); // Red Fill
            cv::polylines(canvas, obs_cv_points, true, cv::Scalar(0, 0, 100), 2); // Dark Outline
        }
        
        // Draw Radius if exists (circular obstacles)
        if (obstacle.get_radius() > 0.0f && !obstacle.get_points().empty()) {
            cv::Point center = worldToImage((float)obstacle.get_points()[0].x, (float)obstacle.get_points()[0].y);
            int radius_px = static_cast<int>(obstacle.get_radius() * scale);
            cv::circle(canvas, center, radius_px, cv::Scalar(0, 0, 255), -1);
        }
    }
    
    for (const auto& victim : victim_list) {
        cv::Point center_px = worldToImage((float)victim.get_center().x, (float)victim.get_center().y);
        
        float raw_radius = victim.get_radius();
        
        // DEBUG: Print the radius to see what is wrong in the logs
        ROS_INFO_THROTTLE(1.0, "[Roadmap] Drawing victim at (%.1f, %.1f) with radius: %.2f", 
                         victim.get_center().x, victim.get_center().y, raw_radius);

        // Convert mm to meters by dividing by 1000.0f
        int radius_px = static_cast<int>((raw_radius / 1000.0f) * scale);
        
        // SAFETY 2: Clamp pixel size
        // Min: 5 pixels (so you can see it)
        // Max: 100 pixels (so it doesn't cover the screen)
        int final_radius = std::max(5, std::min(radius_px, 100));

        // Draw Filled Circle
        cv::circle(canvas, center_px, final_radius, cv::Scalar(128, 0, 128), -1); // Purple
        // Draw Outline
        cv::circle(canvas, center_px, final_radius, cv::Scalar(64, 0, 64), 2);
    }

    // Draw Gates
    for (const auto& gate : gate_list) {
        cv::Point gate_pos = worldToImage((float)gate.get_position().x, (float)gate.get_position().y);
        const auto& q = gate.get_orientation();
        
        // Quaternion to Yaw
        float yaw = std::atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
        
        int arrow_len = 40;
        cv::Point arrow_end(
            gate_pos.x + (int)(arrow_len * std::cos(yaw)),
            gate_pos.y - (int)(arrow_len * std::sin(yaw)) // -sin because Image Y is inverted
        );
        cv::arrowedLine(canvas, gate_pos, arrow_end, cv::Scalar(0, 255, 0), 3, 8, 0, 0.3);
        cv::circle(canvas, gate_pos, 5, cv::Scalar(0, 200, 0), -1);
    }

    // Add Legend
    cv::putText(canvas, "Borders (Black) | Obstacles (Red) | Victims (Purple) | Gates (Green)", 
                cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(50, 50, 50), 2);

    // 6. SAVE IMAGE 
    std::string filename = "/home/ubuntu/trento_lab_home/roadmap_visualization.png";
    try {
        bool success = cv::imwrite(filename, canvas);
        if (success) {
            ROS_INFO("[Roadmap] Image successfully saved to: %s", filename.c_str());
        } else {
            ROS_ERROR("[Roadmap] Failed to save image to %s. Check permissions.", filename.c_str());
        }
    } catch (const cv::Exception& ex) {
        ROS_ERROR("[Roadmap] OpenCV Exception saving: %s", ex.what());
    }

    // 7. DISPLAY IMAGE
    // Using namedWindow ensures the window is created properly before showing
    // We wrap this in a try-catch because in headless docker containers this can crash the node
    try {
        std::string win_name = "Roadmap Visualization";
        
        // 1. Create the window
        cv::namedWindow(win_name, cv::WINDOW_NORMAL); 
        
        // 2. CRITICAL: Force the window size. 
        // Without this, WINDOW_NORMAL often starts as a 1x1 black pixel or collapsed window.
        cv::resizeWindow(win_name, 800, 600); 

        // 3. Show the image
        cv::imshow(win_name, canvas);
        
        // 4. Wait 5 seconds to keep window open
        // Press any key to close earlier
        ROS_INFO("[Roadmap] Visualization window open. Press any key to close...");
        cv::waitKey(5000); 
        
    } catch (const cv::Exception& e) {
        ROS_WARN("[Roadmap] OpenCV Window error (X11 may not be available): %s", e.what());
    }
}

*/

