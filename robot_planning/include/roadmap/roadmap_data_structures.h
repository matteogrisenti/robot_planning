#ifndef ROADMAP_H
#define ROADMAP_H

#include <vector>
#include <unordered_map>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>
#include <memory>

#include "map/map_data_structures.h"

// Simple 2D point structure
struct Vertex {
    double x, y;
    
    Vertex(double x = 0, double y = 0);
    double distance(const Vertex& other) const;
};

// Edge structure with weight (distance)
struct Edge {
    int targetVertex;       // id of the destination vertex
    double weight;
    
    Edge(int target, double w);
};

// Helper structures and methods
struct Cell {
    double minX, minY, maxX, maxY;
    bool isFree;
    Vertex center;
    
    Cell(double x1, double y1, double x2, double y2) 
        : minX(x1), minY(y1), maxX(x2), maxY(y2), isFree(true) {
        center = Vertex((x1 + x2) / 2, (y1 + y2) / 2);
    }
    
    bool contains(const Vertex& v) const {
        return v.x >= minX && v.x <= maxX && v.y >= minY && v.y <= maxY;
    }
};

struct Trapezoid {
        // The vertical walls (x-coordinates)
        double leftX, rightX;

        // The slanted ceiling and floor (y-coordinates)
        double topLeftY, topRightY;       
        double bottomLeftY, bottomRightY; 

        Vertex center;
        std::vector<int> neighbors; // Indices of adjacent trapezoids in the main vector

        // Constructor
        Trapezoid(double lx, double rx, double tly, double try_, double bly, double bry)
            : leftX(lx), rightX(rx), 
              topLeftY(tly), topRightY(try_), 
              bottomLeftY(bly), bottomRightY(bry),
              center(0,0) // Initialize dummy
        {
            // Average of the 4 corners for the center node
            center = Vertex((lx + rx) / 2.0, (tly + try_ + bly + bry) / 4.0);
        }
        
        // Default constructor for vector resizing
        Trapezoid() : leftX(0), rightX(0), topLeftY(0), topRightY(0), 
                      bottomLeftY(0), bottomRightY(0), center(0,0) {}
    };


class Roadmap {
private:
    std::vector<Vertex> vertices;
    std::vector<std::vector<Edge>> adjacencyList; // edges for each vertex

    // Pointer to the Map on which the Roadmap is created
    const Map* linkedMap;
    
    // Helper: check if edge is collision-free (placeholder)
    bool isEdgeValid(const Vertex& p1, const Vertex& p2) const;
    
    // Helper: find k nearest neighbors
    std::vector<int> findKNearestNeighbors(int vertexIdx, int k) const;

public:
    std::shared_ptr<std::vector<Trapezoid>> debugTrapezoids;
    std::shared_ptr<std::vector<Cell>> debugCells;

    Roadmap();
    
    // Add a vertex and return its index
    int addVertex(const Vertex& p);
    
    // Add an edge (optionally bidirectional)
    void addEdge(int from, int to, bool bidirectional = true);

    // Methods to handle the Map pointer
    void setMap(const Map* map_ptr);
    const Map* getMap() const;
    
    // Get number of vertices
    int getNumVertices() const;
    
    // Get vertex position
    const Vertex& getVertex(int idx) const; 
    
    // Get edges from a vertex
    const std::vector<Edge>& getEdges(int vertexIdx) const;

    // plot the Roadmap
    void plot(bool display, bool save, std::string output_path = "");
};

#endif // ROADMAP_H