#ifndef MAP_VISUALIZATION_H
#define MAP_VISUALIZATION_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "roadmap/data_structures.h"

namespace map_viz {

    // Configuration structure
    struct VizConfig {
        int img_width = 1600;
        int img_height = 1600;
        int margin = 50;
        
        // Colors (BGR format)
        cv::Scalar color_background = cv::Scalar(255, 255, 255);    // white
        cv::Scalar color_border = cv::Scalar(0, 0, 0);              // black
        cv::Scalar color_obstacle_fill = cv::Scalar(0, 0, 255);     // red
        cv::Scalar color_obstacle_outline = cv::Scalar(0, 0, 100);  
        cv::Scalar color_victim_fill = cv::Scalar(128, 0, 128);     // purple
        cv::Scalar color_victim_outline = cv::Scalar(64, 0, 64);
        cv::Scalar color_gate = cv::Scalar(0, 255, 0);              // green
        cv::Scalar color_start_fill = cv::Scalar(255, 0, 0);        // yellow
        cv::Scalar color_start_outline = cv::Scalar(200, 0, 0);
        
        // Sizes
        int border_thickness = 3;
        int obstacle_thickness = 2;
        int gate_arrow_length = 40;
        int start_circle_radius = 10;
        int start_arrow_length = 25;
        
        // Window name
        std::string window_name = "Map Visualization";
    };

    // Bounds calculator
    struct WorldBounds {
        float min_x, max_x, min_y, max_y;
        float scale;
        bool points_found;
        
        WorldBounds() : min_x(0), max_x(0), min_y(0), max_y(0), 
                        scale(1.0f), points_found(false) {}
    };

    // Main visualization class
    class MapVisualizer {
    private:
        VizConfig config_;
        cv::Mat canvas_;
        WorldBounds bounds_;
        
        // Helper functions
        void calculateBounds(const Map& map);
        cv::Point worldToImage(float x, float y) const;
        float quaternionToYaw(const Orientation& q) const;
        
        // Drawing functions
        void drawBorders(const Borders& borders);
        void drawObstacles(const Obstacles& obstacles);
        void drawVictims(const Victims& victims);
        void drawGates(const Gates& gates);
        void drawStart(const Start& start);
        void drawLegend();
        void drawGrid();
        
    public:
        MapVisualizer(const VizConfig& config = VizConfig());
        
        // Main render function
        void render(const Map& map);
        
        // Display/Save functions
        void display();
        bool saveToFile(const std::string& filename);
        
        // Get the rendered canvas
        const cv::Mat& getCanvas() const { return canvas_; }
    };

} 

#endif 
