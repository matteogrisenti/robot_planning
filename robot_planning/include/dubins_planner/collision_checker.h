#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

#include "map/map_data_structures.h"
#include "dubins_planner/dubins_trajectory.h"

// --- Strutture Dati di Base ---

struct Point2D {
    double x;
    double y;
};

// Ostacolo Cilindrico/Circolare
struct CircleObstacle {
    Point2D center;
    double radius;
};

// Ostacolo Poligonale
// IMPORTANTE: Si assume che centroid e bounding_radius siano PRE-CALCOLATI
// dal produttore dei dati (es. nel nodo che riceve la mappa).
struct PolygonObstacle {
    std::vector<Point2D> vertices; // Vertici ordinati (es. senso antiorario)
    Point2D centroid;              // Centro per la Broad Phase
    double bounding_radius;        // Distanza massima centro-vertice per la Broad Phase
};


// --- Classe Collision Checker ---
class CollisionChecker {
public:
    /**
     * @brief Costruttore. Configura le dimensioni fisse del robot.
     * @param robot_radius Raggio fisico del robot.
     * @param safety_margin Margine di sicurezza aggiuntivo (opzionale, default 0).
     */
    CollisionChecker(double robot_radius, double safety_margin = 0.0);

    /**
     * @brief Funzione principale di verifica collisioni.
     * Verifica la posa del robot contro un'istantanea degli ostacoli correnti.
     * * @param robot_pose Posizione centrale del robot (x,y).
     * @param circles Lista degli ostacoli circolari correnti.
     * @param polygons Lista degli ostacoli poligonali correnti (con dati broad phase precalcolati).
     * @return true se in collisione, false altrimenti.
     */
    [[nodiscard]] bool check(const Point2D& robot_pose) const;

    /**
     * @brief Converte ostacoli e bordi nel formato ottimizzato per il CollisionChecker.
     * Da chiamare UNA VOLTA dopo aver ricevuto tutti i messaggi ROS.
     */
    void update_collision_cache(const Map& map);

    /**
     * @brief Controlla se una singola configurazione (x,y) è valida (libera).
     */
    bool is_state_valid(double x, double y) const;

    // Insted to pass al the parameter to recompute the curve, we pass directly the curve with reference
    /**
     * @brief Controlla se è possibile connettere A a B con una curva di Dubins valida.
     * @param start Configurazione di partenza {x, y, theta}
     * @param end Configurazione di arrivo {x, y, theta}
     * @param rho Raggio minimo di curvatura (es. 0.5m)
     * @return true se il percorso esiste ed è libero da collisioni.
     */
    bool is_dubins_path_valid(const int best_word, const dubinscurve_out& curve) const;

    
private:
    // Il raggio totale effettivo (robot + margine)
    double effective_robot_radius_;
    // Il quadrato del raggio effettivo (per ottimizzazione confronti)
    double effective_radius_sq_;

    // Cache ottimizzata (Poligoni e Cerchi pronti per il check)
    std::vector<CircleObstacle> cached_circles_;
    std::vector<PolygonObstacle> cached_polygons_;

    // --- Funzioni Helper Private ---

    // Helper per la distanza al quadrato tra due punti
    [[nodiscard]] double distSq(const Point2D& p1, const Point2D& p2) const {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return dx * dx + dy * dy;
    }

    // Calcola il quadrato della distanza minima da un punto P a un segmento AB.
    // Cuore matematico della Narrow Phase.
    double distToSegmentSquared(const Point2D& p, const Point2D& a, const Point2D& b) const;

    // Verifica collisione con un singolo poligono usando la strategia ibrida.
    bool checkSinglePolygon(const Point2D& robot_pose, const PolygonObstacle& poly) const;
};

#endif 
