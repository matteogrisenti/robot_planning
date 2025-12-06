#include "dubins_planner/collision_checker.h"
#include <iostream>

// --- Costruttore ---
CollisionChecker::CollisionChecker(double robot_radius, double safety_margin) {
    if (robot_radius <= 0.0) {
        throw std::invalid_argument("CollisionChecker: Il raggio del robot deve essere > 0");
    }
    if (safety_margin < 0.0) {
        throw std::invalid_argument("CollisionChecker: Il margine di sicurezza non può essere negativo");
    }

    effective_robot_radius_ = robot_radius + safety_margin;
    // Pre-calcoliamo il quadrato per evitare sqrt() nei loop
    effective_radius_sq_ = effective_robot_radius_ * effective_robot_radius_;
}


// --- Funzione Principale di Check ---
bool CollisionChecker::check(const Point2D& robot_pose, 
                             const std::vector<CircleObstacle>& circles,
                             const std::vector<PolygonObstacle>& polygons) const {
    
    // 1. Check Ostacoli Circolari (Veloce ed Esatto)
    for (const auto& obs : circles) {
        // La distanza di collisione è la somma dei raggi
        double collision_dist = effective_robot_radius_ + obs.radius;
        double collision_dist_sq = collision_dist * collision_dist;

        if (distSq(robot_pose, obs.center) < collision_dist_sq) {
            return true; // Collisione rilevata (short-circuit)
        }
    }

    // 2. Check Ostacoli Poligonali (Strategia Ibrida)
    for (const auto& poly : polygons) {
        if (checkSinglePolygon(robot_pose, poly)) {
            return true; // Collisione rilevata
        }
    }

    return false; // Nessuna collisione con nessun ostacolo
}


// --- Implementazione Strategia Ibrida Poligoni ---
bool CollisionChecker::checkSinglePolygon(const Point2D& robot_pose, const PolygonObstacle& poly) const {
    
    // --- FASE 1: BROAD PHASE (Filtro Rapido) ---
    // Usiamo i cerchi circoscritti. Se i cerchi non si toccano, è impossibile che
    // il robot tocchi il poligono.
    
    // Raggio del cerchio circoscritto all'ostacolo
    double poly_bounding_rad = poly.bounding_radius;
    
    // Distanza alla quale i due cerchi (robot e bounding box ostacolo) si toccherebbero
    double broad_threshold = effective_robot_radius_ + poly_bounding_rad;
    double broad_threshold_sq = broad_threshold * broad_threshold;

    // Distanza reale al quadrato tra robot e centroide ostacolo
    double dist_to_centroid_sq = distSq(robot_pose, poly.centroid);

    // Se siamo lontani, saltiamo questo poligono. Grande risparmio di calcoli.
    if (dist_to_centroid_sq > broad_threshold_sq) {
        return false; 
    }

    // --- FASE 2: NARROW PHASE (Controllo Preciso) ---
    // Se siamo arrivati qui, la Broad Phase ha detto "forse".
    // Dobbiamo controllare la distanza precisa punto-segmento per ogni lato.

    if (poly.vertices.size() < 2) return false; // Non è un poligono valido

    for (size_t i = 0; i < poly.vertices.size(); ++i) {
        // Vertice corrente
        const Point2D& v1 = poly.vertices[i];
        // Vertice successivo (con modulo per chiudere il loop dall'ultimo al primo)
        const Point2D& v2 = poly.vertices[(i + 1) % poly.vertices.size()];

        // Calcola distanza al quadrato dal centro robot al segmento v1-v2
        double d2 = distToSegmentSquared(robot_pose, v1, v2);

        // Se la distanza al quadrato è minore del raggio robot al quadrato, collisione.
        if (d2 < effective_radius_sq_) {
            return true;
        }
    }

    // Se abbiamo controllato tutti i lati e nessuno tocca:
    return false;
}

// --- Matematica Narrow Phase ---
// Calcola il quadrato della distanza minima dal punto P al segmento definito da A e B.
double CollisionChecker::distToSegmentSquared(const Point2D& p, const Point2D& a, const Point2D& b) const {
    // Lunghezza al quadrato del segmento AB
    double l2 = distSq(a, b); 
    
    // Caso degenere: A e B sono lo stesso punto
    if (l2 == 0.0) return distSq(p, a); 

    // Consideriamo la retta passante per A e B parametrizzata come: A + t * (B - A).
    // Troviamo la proiezione del punto P su questa retta.
    // Il parametro 't' della proiezione è dato dal prodotto scalare:
    // t = dot(P-A, B-A) / |B-A|^2
    double t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;

    // Clampiamo 't' nell'intervallo [0, 1] per assicurarci che il punto più vicino
    // cada DENTRO il segmento, non sulla retta infinita esterna.
    // Se t < 0, il punto più vicino è A. Se t > 1, è B. Altrimenti è la proiezione.
    t = std::max(0.0, std::min(1.0, t));

    // Calcoliamo le coordinate del punto di proiezione (o estremo) sul segmento
    Point2D projection_point = {
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y)
    };

    // Ritorniamo la distanza al quadrato tra P e il punto trovato sul segmento
    return distSq(p, projection_point);
}