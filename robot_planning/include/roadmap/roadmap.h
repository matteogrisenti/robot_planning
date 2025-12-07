#include <vector>
#include <unordered_map>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>

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

class Roadmap {
private:
    std::vector<Vertex> vertices;
    std::vector<std::vector<Edge>> adjacencyList; // edges for each vertex
    
    // Helper: check if edge is collision-free (placeholder)
    bool isEdgeValid(const Vertex& p1, const Vertex& p2) const;
    
    // Helper: find k nearest neighbors
    std::vector<int> findKNearestNeighbors(int vertexIdx, int k) const;

public:
    Roadmap();
    
    // Add a vertex and return its index
    int addVertex(const Vertex& p);
    
    // Add an edge (optionally bidirectional)
    void addEdge(int from, int to, bool bidirectional = true);
    
    // Get number of vertices
    int getNumVertices() const;
    
    // Get vertex position
    const Vertex& getVertex(int idx) const; 
    
    // Get edges from a vertex
    const std::vector<Edge>& getEdges(int vertexIdx) const;
};