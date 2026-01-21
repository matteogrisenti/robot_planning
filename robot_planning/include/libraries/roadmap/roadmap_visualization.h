#ifndef ROADMAP_VISUALIZATION_H
#define ROADMAP_VISUALIZATION_H

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include "map_library/map_data_structures.h"
#include "libraries/roadmap/roadmap_data_structures.h"

namespace roadmap_viz {

    // Configuration structure
    struct RoadmapVizConfig {
        int img_width = 1600;
        int img_height = 1600;
        int margin = 50;
        
        cv::Scalar color_background = cv::Scalar(255, 255, 255);      // White
        cv::Scalar color_border = cv::Scalar(80, 80, 80);             // Dark Gray
        cv::Scalar color_obstacle_fill = cv::Scalar(100, 100, 100);   // Gray
        cv::Scalar color_obstacle_outline = cv::Scalar(60, 60, 60);   

        cv::Scalar color_victim_fill = cv::Scalar(255, 0, 255);       // Magenta
        cv::Scalar color_victim_outline = cv::Scalar(100, 0, 100);    // Dark Magenta
        
        cv::Scalar color_gate = cv::Scalar(0, 200, 0);                // Green
        int gate_arrow_length = 40;

        cv::Scalar color_vertex = cv::Scalar(255, 255, 0);            // Cyan/Electric Blue (BGR)
        cv::Scalar color_edge = cv::Scalar(255, 200, 0);              // Blueish
        
        int border_thickness = 3;
        int obstacle_thickness = 2;
        int vertex_radius = 6;                 
        int edge_thickness = 2;              
        
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
        
        void calculateBounds(const Map& map);
        cv::Point worldToImage(float x, float y) const;
        cv::Point vertexToImage(const Vertex& v) const;
        
        float quaternionToYaw(const Orientation& q) const;

        void drawBorders(const Borders& borders);
        void drawObstacles(const Obstacles& obstacles);
        
        void drawVictims(const Victims& victims);
        void drawGates(const Gates& gates);

        void drawTrapezoids(const Roadmap& roadmap);
        void drawCells(const Roadmap& roadmap);
        void drawRoadmapEdges(const Roadmap& roadmap);
        void drawRoadmapVertices(const Roadmap& roadmap);
        void drawInfo(const Roadmap& roadmap);
        
    public:
        RoadmapVisualizer(const RoadmapVizConfig& config = RoadmapVizConfig());
        
        void render(const Map& map, const Roadmap& roadmap);

        void drawPath(const Roadmap& roadmap, const std::vector<int>& path);
        
        void display();
        bool saveToFile(const std::string& filename);
        
        const cv::Mat& getCanvas() const { return canvas_; }
    };

} // namespace roadmap_viz

#endif // ROADMAP_VISUALIZATION_H