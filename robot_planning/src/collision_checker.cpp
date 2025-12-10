#include "dubins_planner/collision_checker.h"
#include <iostream>
#include <ros/ros.h>

#include "map/map_data_structures.h"

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
// MERGE CHANGE 
// Since now the cache of cached_circles_ , cached_polygons_ are attribute of the calss they no more need 
// to be passed as parameters

// bool CollisionChecker::check(const Point2D& robot_pose, 
//                             const std::vector<CircleObstacle>& circles,
//                             const std::vector<PolygonObstacle>& polygons) const {

bool CollisionChecker::check(const Point2D& robot_pose) const{
    // 1. Check Ostacoli Circolari (Veloce ed Esatto)
    for (const auto& obs : cached_circles_) {
        // La distanza di collisione è la somma dei raggi
        double collision_dist = effective_robot_radius_ + obs.radius;
        double collision_dist_sq = collision_dist * collision_dist;

        if (distSq(robot_pose, obs.center) < collision_dist_sq) {
            return true; // Collisione rilevata (short-circuit)
        }
    }

    // 2. Check Ostacoli Poligonali (Strategia Ibrida)
    for (const auto& poly : cached_polygons_) {
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




// MERGED CHANEGE
// Since this method have been moved from Roadmap to CollisionChecker also their implementation
// have been moved

// MERGED CHANGE 
// Since the method is not more inside the Roadmap, we need to pass the reference to the map to have
// access to the borders & obstacle information
void CollisionChecker::update_collision_cache(const Map& map) {
    cached_circles_.clear();
    cached_polygons_.clear();

    // ROS_INFO("[Roadmap] Baking collision data...");

    // 1. Converti OSTACOLI DINAMICI
    const auto& obs_vec = map.obstacles.get_obstacles();
    for (const auto& obs : obs_vec) {
        const auto& pts = obs.get_points();
        float r = obs.get_radius();

        // CILINDRI
        if (r > 0 && pts.size() >= 1) {
            cached_circles_.push_back({{pts[0].x, pts[0].y}, (double)r});
        }
        // BOX / POLIGONI
        else if (pts.size() > 2) {
            std::vector<Point2D> verts;
            double sum_x = 0, sum_y = 0;
            for (const auto& p : pts) {
                verts.push_back({p.x, p.y});
                sum_x += p.x; sum_y += p.y;
            }
            
            // Calcolo Centroide e Bounding Radius per CollisionChecker
            Point2D centroid = {sum_x / verts.size(), sum_y / verts.size()};
            double max_dist_sq = 0;
            for (const auto& v : verts) {
                double d2 = std::pow(v.x - centroid.x, 2) + std::pow(v.y - centroid.y, 2);
                if (d2 > max_dist_sq) max_dist_sq = d2;
            }
            cached_polygons_.push_back({verts, centroid, std::sqrt(max_dist_sq)});
        }
    }

    // 2. Converti BORDI MAPPA (come Poligono Statico)
    const auto& border_pts = map.borders.get_points();
    if (border_pts.size() > 2) {
        std::vector<Point2D> verts;
        double sum_x = 0, sum_y = 0;
        for (const auto& p : border_pts) {
            verts.push_back({p.x, p.y});
            sum_x += p.x; sum_y += p.y;
        }
        Point2D centroid = {sum_x / verts.size(), sum_y / verts.size()};
        double max_dist_sq = 0;
        for (const auto& v : verts) {
            double d2 = std::pow(v.x - centroid.x, 2) + std::pow(v.y - centroid.y, 2);
            if (d2 > max_dist_sq) max_dist_sq = d2;
        }
        cached_polygons_.push_back({verts, centroid, std::sqrt(max_dist_sq)});
    }
    
    ROS_INFO("[CollisionChecker] Cache Updated: %lu Circles, %lu Polygons (incl. Borders)", 
             cached_circles_.size(), cached_polygons_.size());
}



// --- Check Puntuale ---
bool CollisionChecker::is_state_valid(double x, double y) const {
    // Ritorna TRUE se il punto è LIBERO (CollisionChecker.check ritorna true se COLLIDE)
    // return !collision_checker_.check({x, y}, cached_circles_, cached_polygons_);

    // MERGE CHANGE
    /* Since now this is a method of the Collisio Checker class it call directly the method */
    return !check({x,y});
}

// MERGE CHANGE
/* I add as parameter the same subins curve reference; insted to recompute a second time inside the
this method */
// bool CollisionChecker::is_dubins_path_valid(const Point& start, double start_theta, 
//                                    const Point& end, double end_theta, 
//                                    double rho, const dubinscurve_out& curve) const {
    
    // MERGE CHANGE: now the curve is passed as reference 
    // dubinscurve_out curve;

    // Configura parametri Dubins
    // double k_max = 1.0 / rho;
    // int best_word = -1;

    // 1. Calcola la curva
    // dubins_shortest_path(start.x, start.y, start_theta, 
    //                      end.x, end.y, end_theta, 
    //                      k_max, best_word, &curve);


bool CollisionChecker::is_dubins_path_valid(const int best_word, const dubinscurve_out& curve) const {
    if (best_word < 0) return false; // Percorso impossibile

    // 2. Campiona e valida ogni punto lungo la curva
    double step_size = 0.1; // Check ogni 10cm
    
    auto check_arc = [&](const dubinsarc_out& arc) -> bool {
        int steps = std::ceil(arc.l / step_size);
        for (int i = 0; i <= steps; ++i) {
            double s = (i * step_size > arc.l) ? arc.l : i * step_size;
            long double x, y, th;
            circline(s, arc.x0, arc.y0, arc.th0, arc.k, x, y, th);

            // Se anche un solo punto collide, il percorso è invalido
            if (!is_state_valid((double)x, (double)y)) {
                return false; 
            }
        }
        return true;
    };

    if (!check_arc(curve.a1)) return false;
    if (!check_arc(curve.a2)) return false;
    if (!check_arc(curve.a3)) return false;

    return true; // Percorso pulito
}
