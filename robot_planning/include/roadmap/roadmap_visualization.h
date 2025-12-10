#ifndef ROADMAP_VISUALIZATION_H
#define ROADMAP_VISUALIZATION_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "map/map_data_structures.h"
#include "roadmap/roadmap_data_structures.h"

namespace roadmap_viz {

    // Configuration structure
    struct RoadmapVizConfig {
        int img_width = 1600;
        int img_height = 1600;
        int margin = 50;
        
        // Colors (BGR format)
        cv::Scalar color_background = cv::Scalar(255, 255, 255);      // white
        cv::Scalar color_border = cv::Scalar(80, 80, 80);             // dark gray
        cv::Scalar color_obstacle_fill = cv::Scalar(100, 100, 100);   // dark gray
        cv::Scalar color_obstacle_outline = cv::Scalar(60, 60, 60);   // darker gray
        cv::Scalar color_vertex = cv::Scalar(255, 255, 0);            // electric blue (BGR: cyan/electric blue)
        cv::Scalar color_edge = cv::Scalar(255, 200, 0);              // electric blue for edges
        
        // Sizes
        int border_thickness = 3;
        int obstacle_thickness = 2;
        int vertex_radius = 8;                 // big dot for vertices
        int edge_thickness = 2;                // line thickness for edges
        
        // Window name
        std::string window_name = "Roadmap Visualization";
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
    class RoadmapVisualizer {
    private:
        RoadmapVizConfig config_;
        cv::Mat canvas_;
        WorldBounds bounds_;
        
        // Helper functions
        void calculateBounds(const Map& map);
        cv::Point worldToImage(float x, float y) const;
        cv::Point vertexToImage(const Vertex& v) const;
        
        // Drawing functions
        void drawBorders(const Borders& borders);
        void drawObstacles(const Obstacles& obstacles);
        void drawTrapezoids(const Roadmap& roadmap);
        void drawCells(const Roadmap& roadmap);
        void drawRoadmapEdges(const Roadmap& roadmap);
        void drawRoadmapVertices(const Roadmap& roadmap);
        void drawInfo(const Roadmap& roadmap);
        
    public:
        RoadmapVisualizer(const RoadmapVizConfig& config = RoadmapVizConfig());
        
        // Main render function
        void render(const Map& map, const Roadmap& roadmap);
        
        // Display/Save functions
        void display();
        bool saveToFile(const std::string& filename);
        
        // Get the rendered canvas
        const cv::Mat& getCanvas() const { return canvas_; }
    };

} // namespace roadmap_viz

#endif // ROADMAP_VISUALIZATION_H
