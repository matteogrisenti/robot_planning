#ifndef ROADMAP_H
#define ROADMAP_H

#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <memory>
#include <iostream>

#include "map/map_data_structures.h"

// --- VERTEX ESTESO ---
struct Vertex {
    double x, y;
    double theta; // Nuovo campo per Dubins
    
    // COMPATIBILITY: Default theta=0.0. 
    // Il combinatorial planning chiamerà Vertex(x, y) e funzionerà come prima.
    Vertex(double x = 0, double y = 0, double theta = 0.0) 
        : x(x), y(y), theta(theta) {}
    
    // Distanza Euclidea (usata da tutti per k-NN)
    double distance(const Vertex& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx*dx + dy*dy);
    }
};

// --- EDGE ESTESO ---
struct Edge {
    int targetVertex;
    double weight;
    
    // Nuovo campo: Tipo di manovra Dubins (0-5) o -1 per segmento dritto
    int dubinsType; 
    
    // COMPATIBILITY: Default type = -1 (Segmento dritto).
    Edge(int target, double w, int type = -1) 
        : targetVertex(target), weight(w), dubinsType(type) {}
};

// ... Cell e Trapezoid rimangono invariati (omessi per brevità, copiali dal tuo file) ...
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
    double leftX, rightX, topLeftY, topRightY, bottomLeftY, bottomRightY; 
    Vertex center;
    std::vector<int> neighbors;
    Trapezoid(double lx, double rx, double tly, double try_, double bly, double bry)
        : leftX(lx), rightX(rx), topLeftY(tly), topRightY(try_), bottomLeftY(bly), bottomRightY(bry), center(0,0) {
        center = Vertex((lx + rx) / 2.0, (tly + try_ + bly + bry) / 4.0);
    }
    Trapezoid() : leftX(0), rightX(0), topLeftY(0), topRightY(0), bottomLeftY(0), bottomRightY(0), center(0,0) {}
};

class Roadmap {
private:
    std::vector<Vertex> vertices;
    std::vector<std::vector<Edge>> adjacencyList; 
    const Map* linkedMap;
    
    // Metodo helper interno per trovare vicini
    std::vector<int> findNeighborsInternal(int vertexIdx, double radius = -1, int k = -1) const {
        std::vector<std::pair<double, int>> candidates;
        for (int i = 0; i < vertices.size(); i++) {
            if (i == vertexIdx) continue;
            double dist = vertices[vertexIdx].distance(vertices[i]);
            
            // Filtro raggio (se radius > 0)
            if (radius > 0 && dist > radius) continue;
            
            candidates.push_back({dist, i});
        }

        // Se serve K-Nearest, ordiniamo e tagliamo
        if (k > 0) {
            std::sort(candidates.begin(), candidates.end());
            if (candidates.size() > k) candidates.resize(k);
        }

        std::vector<int> result;
        for(const auto& p : candidates) result.push_back(p.second);
        return result;
    }

public:
    std::shared_ptr<std::vector<Trapezoid>> debugTrapezoids;
    std::shared_ptr<std::vector<Cell>> debugCells;

    Roadmap(); 
    
    int addVertex(const Vertex& p);
    
    // COMPATIBILITY: Questo metodo esiste già nel tuo cpp
    void addEdge(int from, int to, bool bidirectional = true);

    // NUOVO OVERLOAD: Per PRM con peso esplicito e tipo Dubins
    void addEdge(int from, int to, double weight, bool bidirectional, int dubinsType);

    void setMap(const Map* map_ptr);
    const Map* getMap() const;
    
    int getNumVertices() const;
    const Vertex& getVertex(int idx) const; 
    const std::vector<Edge>& getEdges(int vertexIdx) const;

    // Helper esposti per i planner
    std::vector<int> getNearestNeighbors(int idx, double radius) const {
        return findNeighborsInternal(idx, radius, -1);
    }
    
    // Metodo usato dal vecchio codice (presumo)
    std::vector<int> findKNearestNeighbors(int vertexIdx, int k) const {
        return findNeighborsInternal(vertexIdx, -1, k);
    }

    void plot(bool display, bool save, std::string output_path = "");
};

#endif // ROADMAP_H