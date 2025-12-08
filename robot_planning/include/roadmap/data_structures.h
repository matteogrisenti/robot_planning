#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <vector>
#include <iostream> // for std::cout
#include <iomanip>  // for std::setprecision


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
class Borders {
private:
    std::vector<Point> points;  // Vertex of the Poligonal Shape

public:
    Borders();

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
    Start(); 
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

    // Get axis-aligned bounding box
    void get_bounding_box(double& minX, double& minY, double& maxX, double& maxY) const; 

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


// --------------- Complete Map ----------------
class Map {
public:
    Borders borders;
    Gates gates;
    Start start; 
    Obstacles obstacles;
    Victims victims;

    Map();

    void paint_map();

    // Get map bounding box
    void get_bounding_box(double& minX, double& minY, double& maxX, double& maxY) const; 
};



#endif // DATA STRUCTURES