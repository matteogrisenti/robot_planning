#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/MarkerArray.h>

// Includiamo l'header principale che aggrega tutto (CollisionChecker, Dubins, Strutture)
#include "roadmap/data_structures.h"

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
    
    // Subscriber
    ros::Subscriber sub_borders, sub_gates, sub_obs, sub_victims;
    // Publisher Debug
    ros::Publisher pub_markers;

    // Logica Core
    Roadmap roadmap;
    
    // Stato del nodo
    bool map_ready = false;
    struct {
        bool borders = false;
        bool gates = false;
        bool obstacles = false;
        bool victims = false;
    } received;

    Point goal_pos;
    double goal_theta = 0.0;

public:
    DataVerifierNode() : roadmap(0.25, 0.05) { // Raggio robot 0.25, Safety 0.05
        
        // Topic setup
        sub_borders = nh.subscribe("/map_borders", 1, &DataVerifierNode::cbBorders, this);
        sub_gates   = nh.subscribe("/gates", 1, &DataVerifierNode::cbGates, this);
        sub_obs     = nh.subscribe("/obstacles", 1, &DataVerifierNode::cbObstacles, this);
        sub_victims = nh.subscribe("/victims", 1, &DataVerifierNode::cbVictims, this);
        
        pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/verifier/debug_markers", 1);

        ROS_INFO("DataVerifierNode avviato. In attesa della mappa...");
    }

    // --- CALLBACKS: Riempiono solo le strutture dati ---

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

    // --- LOGICA DI COSTRUZIONE ---
    
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
            // 3. Verifica Collisioni sulla path (Collision Checker)
            logic_valid = roadmap.is_dubins_path_valid(start, start_theta, goal_pos, goal_theta, rho);
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
                geometry_msgs::Point p; p.x=x; p.y=y; ma.markers.push_back(path_m); // Fix scope
                path_m.points.push_back(p);
            }
        };
        add_arc(curve.a1); add_arc(curve.a2); add_arc(curve.a3);
        ma.markers.push_back(path_m);

        // Marker Ostacoli (Debug visuale di cosa vede la roadmap)
        int id = 1;
        for(const auto& obs : roadmap.obstacles.get_obstacles()) {
            visualization_msgs::Marker m;
            m.header.frame_id = "map"; m.ns = "debug_obs"; m.id = id++;
            m.type = visualization_msgs::Marker::LINE_STRIP; m.scale.x = 0.05;
            m.color.r = 1.0; m.color.a = 0.5;
            for(const auto& p : obs.get_points()) {
                geometry_msgs::Point gp; gp.x=p.x; gp.y=p.y;
                m.points.push_back(gp);
            }
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