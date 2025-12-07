#pragma once
#include <vector>
#include <iostream> // for std::cout
#include <iomanip>  // for std::setprecision

#include "dubins_planner/collision_checker.h"
#include "dubins_planner/dubins_trajectory.h"


struct Point {
    double x;
    double y;
    double z;
};

struct Orientation {
    double x;
    double y;
    double z;
    double w;
};



// -------------- MapBorders -----------------
// Bring all the vertices of the Map
class MapBorders {
private:
    std::vector<Point> points;  // Vertex of the Poligonal Shape

public:
    MapBorders();

    const std::vector<Point>& get_points() const;

    void add_point(double x, double y, double z);

    void clear();

    std::string to_string() const;
};


// ---------------- Start Position ---------------------
class Start {
private:
    Point position;
    Orientation orientation;

public: 
    Start(const Point& position, const Orientation& orientation);

    const Point& get_position() const;
    const Orientation& get_orientation() const;

    std::string to_string() const;
};


// ---------------- Gate ---------------------
// Rappresent a single gate
class Gate {
private:
    Point position;             // Position of the gate
    Orientation orientation;    // Orientation of the gate

public:
    Gate(const Point& position, const Orientation& orientation);

    const Point& get_position() const;
    const Orientation& get_orientation() const;

    std::string to_string() const;
};

// Bring all the gates
class Gates {
private:
    std::vector<Gate> gates;    // Vector of Gates

public:
    Gates();

    const std::vector<Gate>& get_gates() const;

    bool add_gate(const Point& position, const Orientation& orientation);

    void clear(); 

    std::string to_string() const;

};


// ---------------- Obstacle ---------------------
// Rapresent a single obstacle
class Obstacle {
private:
    std::vector<Point> points;  // Vertex of the Poligonal Shape
    float radius;               // Radius for circular/point obstacles

public:
    Obstacle(const std::vector<Point>* points_ptr, const float radius);

    const std::vector<Point>& get_points() const;
    const float get_radius() const; 

    std::string to_string() const;
};

// Bring all the obstacles
class Obstacles {
private:
    std::vector<Obstacle> obstacles;    // Vector of the Obstacle

public:
    Obstacles(); 

    const std::vector<Obstacle>& get_obstacles() const;

    bool add_obstacle(const std::vector<Point>* points, const float radius);

    void clear();

    std::string to_string() const;
};


// ----------------- Victim ------------------------
// Rapresent a single victim
class Victim {
private:
    Point center;   // Center of the Victims Circle
    float radius;   // Radius

public:
    Victim(const Point& center, const float radius);

    const Point& get_center() const;
    const float get_radius() const; 

    std::string to_string() const;
};

// Bring all the victims
class Victims {
private:
    std::vector<Victim> victims;    // Vector of the Obstacle

public:
    Victims(); 

    const std::vector<Victim>& get_victims() const;

    bool add_victim(const Point& center, const float radius);

    void clear();

    std::string to_string() const;
};


// --------------- Complete roadmap ----------------
class Roadmap {
public:
    MapBorders mapBorders;
    Gates gates;
    Obstacles obstacles;
    Victims victims;

    // Costruttore: Inizializza il checker
    Roadmap(double robot_radius = 0.25, double safety_margin = 0.05);

    // Metodo esistente
    void paint_roadmap();

    /**
     * @brief Converte ostacoli e bordi nel formato ottimizzato per il CollisionChecker.
     * Da chiamare UNA VOLTA dopo aver ricevuto tutti i messaggi ROS.
     */
    void update_collision_cache();

    /**
     * @brief Controlla se una singola configurazione (x,y) è valida (libera).
     */
    bool is_state_valid(double x, double y) const;

    /**
     * @brief Controlla se è possibile connettere A a B con una curva di Dubins valida.
     * @param start Configurazione di partenza {x, y, theta}
     * @param end Configurazione di arrivo {x, y, theta}
     * @param rho Raggio minimo di curvatura (es. 0.5m)
     * @return true se il percorso esiste ed è libero da collisioni.
     */
    bool is_dubins_path_valid(const Point& start, double start_theta, 
                              const Point& end, double end_theta, 
                              double rho) const;

private:
    // Motore di collisione
    CollisionChecker collision_checker_;

    // Cache ottimizzata (Poligoni e Cerchi pronti per il check)
    std::vector<CircleObstacle> cached_circles_;
    std::vector<PolygonObstacle> cached_polygons_;
};
