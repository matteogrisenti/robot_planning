#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/MarkerArray.h>


// MERGE CHANGE
// Added the module that menage automusly the read and organisation of the data of the map
#include "map/map_builder.h"
#include "roadmap/roadmap_data_structures.h"
#include "dubins_planner/collision_checker.h"
#include "dubins_planner/dubins_trajectory.h"

// --- VARIABILI GLOBALI DUBINS (Necessarie per la libreria legacy C-Style) ---
// Non toccare queste, servono a dubins_trajectory.cpp
bool DEBUG = false;
long double X0, Y0, Th0, Xf, Yf, Thf, Kmax;
int pidx;
int no_waypts, step, no_of_samples;
long double angle_step;
dubinscurve_out dubin_curve;
point init_pt, final_pt;
std::vector<point> best_path;
// --------------------------------------------------------------------------

class DataVerifierNode {
private:
    ros::NodeHandle nh;

    ros::Publisher pub_markers;

    // MERGE CHANGE
    /* The logic of read the map information has been moved in the map logic: include/map/ */
    // Subscriber
    // ros::Subscriber sub_borders, sub_gates, sub_obs, sub_victims;
    // Stato del nodo
    // bool map_ready = false;
    // struct {
    //     bool borders = false;
    //     bool gates = false;
    //     bool obstacles = false;
    //     bool victims = false;
    // } received;

    Point goal_pos;
    double goal_theta = 0.0;

    Map map_; 
    bool map_ready = false; // Flag to ensure map is loaded

    // MERGE CHANGE
    /* For the new organization of data the roadmap should be only the set of vertex and edges;
    For this reason I thoungh tha was better to extrapolate the collision checker as an external entity
    that is initialized on the same map and check indipendently the collision of path */
    CollisionChecker collision_checker;
    

public:
    DataVerifierNode() : collision_checker(0.25, 0.05){     // Raggio robot 0.25, Safety 0.05
        
        // MERGE CHANGE 
        /* This logic has been sobstituted with the map_builder logic (include/map/map_builder) with implement all this in a library
        make the code more clean */
        // Topic setup 
        // sub_borders = nh.subscribe("/map_borders", 1, &DataVerifierNode::cbBorders, this);
        // sub_gates   = nh.subscribe("/gates", 1, &DataVerifierNode::cbGates, this);
        // sub_obs     = nh.subscribe("/obstacles", 1, &DataVerifierNode::cbObstacles, this);
        // sub_victims = nh.subscribe("/victims", 1, &DataVerifierNode::cbVictims, this);

        // 1. Build the map
        ROS_INFO("Building map...");
        map_builder::MapBuilder builder(nh, 100.0);
        map_ = builder.buildMap();
        map_ready = true;

        // 2. Access the vector of gates
        // Map -> Gates class -> vector<Gate>
        const auto& all_gates = map_.gates.get_gates();

        // 3. Safety Check: Ensure at least one gate exists before accessing [0]
        if (!all_gates.empty()) {
            // Access the first gate -> get_position() -> x/y
            const Point& first_gate_pos = all_gates[0].get_position();

            goal_pos.x = first_gate_pos.x;
            goal_pos.y = first_gate_pos.y;
            
            // If you need the orientation (theta) as well:
            const Orientation& o = all_gates[0].get_orientation();
            goal_theta = std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z));
        } else {
            ROS_WARN("No gates found in the map! Defaulting to (0,0)");
            goal_pos.x = 0.0;
            goal_pos.y = 0.0;
        }

        // MERGE CHANGE
        /* AS the Checher is an autonom entity it need to be initilized separatley; before it was made
        insede the constructor of the Roadmap class, now it is no more possible */
        ROS_INFO("Initilize the Collision Checker...");                 
        collision_checker.update_collision_cache(map_);   // Intiliazize the checker with the map borders and obstacles
         
        pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/verifier/debug_markers", 1);
        // ROS_INFO("DataVerifierNode avviato. In attesa della mappa...");
    }

    /* --- CALLBACKS: Riempiono solo le strutture dati ---
    This code is all implemented in the MapBuilder class in the map/map_builder.h include file
    void cbBorders(const geometry_msgs::Polygon::ConstPtr& msg) {
        if (received.borders) return;
        for (const auto& p : msg->points) roadmap.mapBorders.add_point(p.x, p.y, p.z);
        received.borders = true;
        checkBuild();
    }

    void cbGates(const geometry_msgs::PoseArray::ConstPtr& msg) {
        if (received.gates) return;
        for (const auto& pose : msg->poses) {
            Point p = {pose.position.x, pose.position.y, pose.position.z};
            Orientation o = {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
            roadmap.gates.add_gate(p, o);

            // Usiamo il primo gate come target di test
            if (roadmap.gates.get_gates().size() == 1) {
                goal_pos = p;
                // Yaw da Quaternione
                goal_theta = std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z));
            }
        }
        received.gates = true;
        checkBuild();
    }

    void cbObstacles(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        if (received.obstacles) return;
        for (const auto& obs : msg->obstacles) {
            std::vector<Point> pts;
            for (const auto& p : obs.polygon.points) pts.push_back({p.x, p.y, p.z});
            roadmap.obstacles.add_obstacle(&pts, obs.radius);
        }
        received.obstacles = true;
        checkBuild();
    }

    void cbVictims(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        if (received.victims) return;
        for (const auto& v : msg->obstacles) {
            if (v.polygon.points.empty()) continue;
            Point c = {v.polygon.points[0].x, v.polygon.points[0].y, v.polygon.points[0].z};
            roadmap.victims.add_victim(c, v.radius);
        }
        received.victims = true;
        checkBuild();
    }
    */


    
    // MERGE CHANGE
    // This is implemente already in the costrucot of the node; line 79: 
    //      collision_checker.update_collision_cache(map)
    /*  --- LOGICA DI COSTRUZIONE ---
    void checkBuild() {
        if (map_ready) return;
        if (received.borders && received.gates && received.obstacles && received.victims) {
            ROS_INFO("Tutti i dati ricevuti. Generazione Collision Cache...");
            // QUi avviene la magia: conversione da ROS raw data a poligoni ottimizzati
            roadmap.update_collision_cache();
            map_ready = true;
            ROS_INFO("Roadmap pronta per la verifica logica.");
        }
    }
    */

    // --- LOOP DI VERIFICA (Running at rate) ---
    void run_verification() {
        if (!map_ready) return;

        // 1. Definiamo scenario di test (Start -> Gate)
        Point start = {0.0, 0.0, 0.0};
        double start_theta = 0.0; // Robot guarda verso X+
        double rho = 0.5; // Raggio curvatura minimo
        
        // 2. Calcolo Dubins (Logica pura)
        int best_idx = -1;
        dubinscurve_out curve;
        dubins_shortest_path(start.x, start.y, start_theta, 
                             goal_pos.x, goal_pos.y, goal_theta, 
                             1.0/rho, best_idx, &curve);
        
        bool logic_valid = false;
        if (best_idx >= 0) {
            // MERGE CHANGE: update the call with the new perameters
            // 3. Verifica Collisioni sulla path (Collision Checker)
            // logic_valid = roadmap.is_dubins_path_valid(start, start_theta, goal_pos, goal_theta, rho);
            logic_valid = collision_checker.is_dubins_path_valid(best_idx, curve);
        }

        // 4. Output Console (Brutale)
        ROS_INFO_THROTTLE(2, "Verifica Logica: Path Dubins L=%.2f | Collision Free: %s", 
                          (double)(best_idx >= 0 ? curve.L : -1.0), 
                          logic_valid ? "SI" : "NO (COLLISIONE)");

        // 5. Visualizzazione Rviz
        publishViz(curve, logic_valid);
    }

    // --- HELPER VISUALIZZAZIONE ---
    
    void publishViz(const dubinscurve_out& curve, bool is_valid) {
        visualization_msgs::MarkerArray ma;
        
        // Marker Path Dubins
        visualization_msgs::Marker path_m;
        path_m.header.frame_id = "map";
        path_m.header.stamp = ros::Time::now();
        path_m.ns = "debug_path";
        path_m.id = 0;
        path_m.type = visualization_msgs::Marker::LINE_STRIP;
        path_m.action = visualization_msgs::Marker::ADD;
        path_m.scale.x = 0.05; 
        path_m.pose.orientation.w = 1.0;
        
        // Colore semaforico: Verde = OK, Rosso = Collisione
        if (is_valid) { path_m.color.r=0.0; path_m.color.g=1.0; path_m.color.a=1.0; }
        else          { path_m.color.r=1.0; path_m.color.g=0.0; path_m.color.a=1.0; }

        // Campionamento path per visualizzazione
        auto add_arc = [&](const dubinsarc_out& arc) {
            double step = 0.1;
            int n = std::ceil(arc.l / step);
            for(int i=0; i<=n; i++) {
                double s = std::min((double)arc.l, i*step);
                long double x, y, th;
                circline(s, arc.x0, arc.y0, arc.th0, arc.k, x, y, th);
                geometry_msgs::Point p; 
                p.x = x; 
                p.y = y; 
                p.z = 0; // Always good to set Z
                
                // ONLY ADD TO POINTS ARRAY HERE
                path_m.points.push_back(p);
            }
        };

        // Add all segments
        add_arc(curve.a1); 
        add_arc(curve.a2); 
        add_arc(curve.a3);
        
        // 3. Push to MarkerArray ONCE at the end
        ma.markers.push_back(path_m);

        // Marker Ostacoli (Debug visuale di cosa vede la roadmap)
        int id = 1;
        for(const auto& obs : map_.obstacles.get_obstacles()) {
            visualization_msgs::Marker m;
            m.header.frame_id = "map"; 
            m.header.stamp = ros::Time::now();
            m.ns = "debug_obs"; 
            m.id = id++;
            m.type = visualization_msgs::Marker::LINE_STRIP; 
            m.action = visualization_msgs::Marker::ADD;
            m.scale.x = 0.05;
            m.color.r = 1.0; m.color.a = 0.5; // Red, semi-transparent
            m.pose.orientation.w = 1.0;

            for(const auto& p : obs.get_points()) {
                geometry_msgs::Point gp; gp.x=p.x; gp.y=p.y;
                m.points.push_back(gp);
            }
            // Close the loop for polygons
            if(!m.points.empty()) m.points.push_back(m.points[0]); // Chiudi loop
            ma.markers.push_back(m);
        }

        pub_markers.publish(ma);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_verifier");
    DataVerifierNode node;
    
    ros::Rate r(5); // 5 Hz
    while(ros::ok()) {
        ros::spinOnce();
        node.run_verification();
        r.sleep();
    }
    return 0;
}