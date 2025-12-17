#include <vector>
#include <unordered_map>
#include <cmath>
#include <random>
#include <algorithm>
#include <limits>
#include <iostream>

#include "roadmap/roadmap_data_structures.h"
#include "roadmap/roadmap_visualization.h"

// NOTA: I costruttori di Vertex e Edge sono definiti inline nel .h 
// o non richiedono implementazione separata se usano liste di inizializzazione nel .h.
// Rimuoviamo le definizioni duplicate dal .cpp per evitare conflitti.

/*================= ROADMAP ===================*/
Roadmap::Roadmap() : linkedMap(nullptr) {}

// Add a vertex and return its index
int Roadmap::addVertex(const Vertex& p) {
    vertices.push_back(p);
    adjacencyList.push_back(std::vector<Edge>());
    return vertices.size() - 1;
}

// METODO 1: Compatibilità Combinatorial (peso Euclideo calcolato auto)
void Roadmap::addEdge(int from, int to, bool bidirectional){
    if (from >= vertices.size() || to >= vertices.size()) return;
    
    // Calcola peso Euclideo
    double weight = vertices[from].distance(vertices[to]);
    
    // Aggiungi edge con type -1 (default)
    adjacencyList[from].push_back(Edge(to, weight, -1));
    
    if (bidirectional) {
        adjacencyList[to].push_back(Edge(from, weight, -1));
    }
}

// METODO 2: Dubins / Sample Based (peso e tipo espliciti)
void Roadmap::addEdge(int from, int to, double weight, bool bidirectional, int dubinsType) {
    if (from < 0 || from >= vertices.size() || to < 0 || to >= vertices.size()) return;
    
    adjacencyList[from].push_back(Edge(to, weight, dubinsType));
    
    if (bidirectional) {
        adjacencyList[to].push_back(Edge(from, weight, dubinsType));
    }
}

bool Roadmap::removeEdge(int from, int to, bool bidirectional) {
    if (from < 0 || from >= vertices.size() || to < 0 || to >= vertices.size()) {
        return false;
    }

    bool found = false;

    // 1. Rimuovi arco from -> to
    auto& edgesFrom = adjacencyList[from];
    auto it = std::remove_if(edgesFrom.begin(), edgesFrom.end(), 
                             [to](const Edge& e){ return e.targetVertex == to; });
    
    if (it != edgesFrom.end()) {
        edgesFrom.erase(it, edgesFrom.end());
        found = true;
    }

    // 2. Rimuovi arco to -> from (se bidirezionale)
    if (bidirectional) {
        auto& edgesTo = adjacencyList[to];
        auto it2 = std::remove_if(edgesTo.begin(), edgesTo.end(), 
                                  [from](const Edge& e){ return e.targetVertex == from; });
        if (it2 != edgesTo.end()) {
            edgesTo.erase(it2, edgesTo.end());
            found = true; // Consideriamo successo se ne troviamo almeno uno
        }
    }

    return found;
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

// Helper interno per trovare vicini (implementazione spostata qui dal .h se necessario, 
// ma se nel .h hai messo "findNeighborsInternal" nel corpo della classe, qui non serve. 
// Dato l'errore "redefinition", sembra che tu abbia definito le funzioni inline nel .h 
// E cercato di ridefinirle qui. 
// SOLUZIONE: Se nel .h le funzioni 'findKNearestNeighbors', 'getNearestNeighborsK' etc. 
// sono definite dentro la classe, NON devono essere qui.
// Se invece nel .h c'è solo il prototipo (senza {}), allora qui serve l'implementazione.

// Basandomi sul codice che ti ho dato prima per il .h, le funzioni getNearestNeighbors erano inline.
// Quindi Rimuovo le implementazioni duplicate da qui per risolvere l'errore di linker/compiler.

// Se 'findKNearestNeighbors' nel .h era dichiarata ma non definita, ecco l'implementazione corretta:
/*
std::vector<int> Roadmap::findKNearestNeighbors(int vertexIdx, int k) const {
    // ... logica ...
}
*/
// Tuttavia, il tuo errore dice "redefinition". Questo significa che il compilatore vede il corpo della funzione due volte.
// Manteniamo questo file pulito. Le funzioni complesse definite nel .h (come findNeighborsInternal) rimangono lì.

void Roadmap::plot(bool display, bool save, std::string output_path) {
    // Create visualizer with config
    roadmap_viz::RoadmapVizConfig viz_config;
    viz_config.vertex_radius = 5; // Un po' più visibile
    roadmap_viz::RoadmapVisualizer viz(viz_config);
    
    // Render
    if (this->linkedMap != nullptr) {
        viz.render(*this->linkedMap, *this);
    } else {
        std::cerr << "Error: Cannot plot roadmap because no Map is linked." << std::endl;
        return;
    }
    
    // Display or save
    if (save && !output_path.empty()) {
        viz.saveToFile(output_path); 
        ROS_INFO("[Roadmap] Roadmap saved to: %s", output_path.c_str());
    }
    if(display){
        viz.display();
    }
}