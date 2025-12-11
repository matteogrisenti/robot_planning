#include <vector>
#include <unordered_map>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>

#include "roadmap/roadmap_data_structures.h"
#include "roadmap/roadmap_visualization.h"



/*================= VERTEX ===================*/
Vertex::Vertex(double x, double y) : x(x), y(y) {}

double Vertex::distance(const Vertex& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    return std::sqrt(dx*dx + dy*dy);
}



/*================= EDGE ===================*/
Edge::Edge(int target, double w) : targetVertex(target), weight(w) {}



/*================= ROADMAP ===================*/
Roadmap::Roadmap() : linkedMap(nullptr) {}

// Add a vertex and return its index
int Roadmap::addVertex(const Vertex& p) {
    vertices.push_back(p);
    adjacencyList.push_back(std::vector<Edge>());
    return vertices.size() - 1;
}

// Add an edge (optionally bidirectional)
void Roadmap::addEdge(int from, int to, bool bidirectional){

    if (from >= vertices.size() || to >= vertices.size()) return;
    
    double weight = vertices[from].distance(vertices[to]);
    adjacencyList[from].push_back(Edge(to, weight));
    
    if (bidirectional) {
        adjacencyList[to].push_back(Edge(from, weight));
    }
}

// NUOVO OVERLOAD per sample_based planning: Aggiunge un arco con un peso esplicito (es. lunghezza Dubins)
void Roadmap::addEdge(int from, int to, double weight, bool bidirectional) {
    // 1. Controllo di sicurezza sugli indici
    if (from < 0 || from >= vertices.size() || to < 0 || to >= vertices.size()) {
        // Opzionale: stampa un errore ROS_ERROR o std::cerr
        return;
    }
    
    // 2. Aggiungi l'arco diretto (from -> to) con il peso fornito
    adjacencyList[from].push_back(Edge(to, weight));
    
    // 3. Gestione opzionale bidirezionalità
    // NOTA: Per i robot non-olonomi (Dubins), 'bidirectional' dovrebbe essere quasi sempre 'false'
    // perché il costo per andare da A a B è diverso da quello per andare da B ad A.
    if (bidirectional) {
        adjacencyList[to].push_back(Edge(from, weight));
    }
}

void Roadmap::setMap(const Map* map_ptr) { this->linkedMap = map_ptr; }
const Map* Roadmap::getMap() const { return this->linkedMap; }

// Get number of vertices
int Roadmap::getNumVertices() const {
    return vertices.size();
}

// Get vertex position
const Vertex& Roadmap::getVertex(int idx) const {
    return vertices[idx];
}

// Get edges from a vertex
const std::vector<Edge>& Roadmap::getEdges(int vertexIdx) const {
    return adjacencyList[vertexIdx];
}
    
// Helper: check if edge is collision-free (placeholder)
bool Roadmap::isEdgeValid(const Vertex& p1, const Vertex& p2) const {
    // TODO: Implement collision checking with obstacles
    // For now, always return true
    return true;
}
    
// Helper: find k nearest neighbors
std::vector<int> Roadmap::findKNearestNeighbors(int vertexIdx, int k) const {
    std::vector<std::pair<double, int>> distances;
    
    for (int i = 0; i < vertices.size(); i++) {
        if (i != vertexIdx) {
            double dist = vertices[vertexIdx].distance(vertices[i]);
            distances.push_back({dist, i});
        }
    }
    
    // Sort by distance and take k nearest
    std::sort(distances.begin(), distances.end());
    
    std::vector<int> neighbors;
    for (int i = 0; i < std::min(k, (int)distances.size()); i++) {
        neighbors.push_back(distances[i].second);
    }
    
    return neighbors;
}

void Roadmap::plot(bool display, bool save, std::string output_path) {
    // Create visualizer with config
    roadmap_viz::RoadmapVizConfig viz_config;
    viz_config.vertex_radius = 10;
    roadmap_viz::RoadmapVisualizer viz(viz_config);
    
    // Render
    if (this->linkedMap != nullptr) {
        viz.render(*this->linkedMap, *this);
    } else {
        // Handle error: Map hasn't been set yet!
        std::cerr << "Error: Cannot plot roadmap because no Map is linked." << std::endl;
        return;
    }
    
    // Display or save
    if (save && !output_path.empty()) {
        // CORRECTED: used 'output_path' instead of undefined 'output_file'
        viz.saveToFile(output_path); 
        ROS_INFO("[RoadmapTest] Roadmap saved to: %s", output_path.c_str());
    }
    if(display){
        viz.display();
    }
}