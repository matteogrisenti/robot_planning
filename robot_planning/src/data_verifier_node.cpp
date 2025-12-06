#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h> // HEADER NUOVO
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>

// --- HEADER PROGETTO ---
#include "dubins_planner/dubins_trajectory.h"
#include "dubins_planner/collision_checker.h" 

// Variabili globali libreria Dubins
bool DEBUG = false; 
long double X0, Y0, Th0, Xf, Yf, Thf, Kmax;
int pidx;
int no_waypts, step, no_of_samples;
long double angle_step;
dubinscurve_out dubin_curve;
point init_pt, final_pt;
std::vector<point> best_path;

// Struttura interna per le vittime
struct Victim {
    double x; double y; double value; int id;
};

class DataVerifier {
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_obs;
    ros::Subscriber sub_victims;
    ros::Subscriber sub_gate;
    ros::Subscriber sub_borders; // NUOVO SUBSCRIBER
    ros::Publisher pub_markers;

    CollisionChecker collision_checker_;
    
    // Dati Dinamici (aggiornati spesso)
    std::vector<CircleObstacle> current_circles_;
    std::vector<PolygonObstacle> current_polygons_;
    std::vector<Victim> current_victims_;

    // Dati Statici (Mappa)
    std::vector<PolygonObstacle> static_borders_; // NUOVO BUFFER BORDI
    bool borders_received_ = false;

    bool debug_mode_;

public:
    DataVerifier() : collision_checker_(0.25, 0.05) { 
        nh.param("debug", debug_mode_, true); 

        pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/debug/environment_markers", 1, true);
        
        sub_obs = nh.subscribe("/obstacles", 1, &DataVerifier::obstaclesCallback, this);
        sub_victims = nh.subscribe("/victims", 1, &DataVerifier::victimsCallback, this);
        sub_gate = nh.subscribe("/gate_position", 1, &DataVerifier::gateCallback, this);
        
        // Sottoscrizione ai bordi (Topic latched solitamente, arriva una volta)
        sub_borders = nh.subscribe("/map_borders", 1, &DataVerifier::bordersCallback, this);
        
        ROS_INFO("--- DATA VERIFIER (BORDERS ENABLED) STARTED ---");
    }

    // --- CALLBACK BORDI (NUOVA) ---
    void bordersCallback(const geometry_msgs::Polygon::ConstPtr& msg) {
        if (msg->points.empty()) return;
        
        static_borders_.clear(); // Reset nel caso arrivi un aggiornamento
        
        std::vector<Point2D> verts;
        double sum_x = 0, sum_y = 0;
        
        // Conversione Geometry -> Point2D
        for (const auto& p : msg->points) {
            verts.push_back({(double)p.x, (double)p.y});
            sum_x += p.x; sum_y += p.y;
        }

        // Calcolo dati Broad Phase (anche se per i bordi serve a poco essendo noi dentro)
        Point2D centroid = {sum_x / verts.size(), sum_y / verts.size()};
        double max_dist_sq = 0;
        for (const auto& v : verts) {
            max_dist_sq = std::max(max_dist_sq, std::pow(v.x - centroid.x, 2) + std::pow(v.y - centroid.y, 2));
        }

        // Aggiungiamo ai bordi statici
        static_borders_.push_back({verts, centroid, std::sqrt(max_dist_sq)});
        borders_received_ = true;

        if (debug_mode_) {
            //ROS_INFO("Mappa ricevuta: %lu vertici. Raggio bounding: %.2fm", verts.size(), std::sqrt(max_dist_sq));
            publishStaticViz(); // Visualizziamo subito i muri blu
        }
    }

    // Visualizza i bordi statici (Muri Blu)
    void publishStaticViz() {
        visualization_msgs::MarkerArray ma;
        int id = 5000;
        for (const auto& border : static_borders_) {
            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = "map";
            line_strip.header.stamp = ros::Time::now();
            line_strip.ns = "map_borders";
            line_strip.id = id++;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.scale.x = 0.1; // Muri più spessi
            line_strip.color.b = 1.0; line_strip.color.g = 0.0; line_strip.color.r = 0.0; // BLU
            line_strip.color.a = 1.0;
            
            for (const auto& v : border.vertices) {
                geometry_msgs::Point p; p.x = v.x; p.y = v.y;
                line_strip.points.push_back(p);
            }
            // Chiudi il poligono
            if (!border.vertices.empty()) {
                geometry_msgs::Point p; p.x = border.vertices[0].x; p.y = border.vertices[0].y;
                line_strip.points.push_back(p);
            }
            ma.markers.push_back(line_strip);
        }
        pub_markers.publish(ma);
    }

    // --- REPORTING (Aggiornato) ---
    std::string getPrimitiveName(int idx) {
        const char* names[] = {"LSL", "RSR", "LSR", "RSL", "RLR", "LRL"};
        if (idx >= 0 && idx < 6) return names[idx];
        return "UNKNOWN";
    }

    void printDebugReport(const dubinscurve_out& curve, int prim_idx, bool safe, 
                          double start[3], double goal[3], 
                          double collision_s, double collision_pt[2]) {
        if (!debug_mode_) return;
        std::stringstream ss; ss << std::fixed << std::setprecision(2);
        ss << "\n\033[1;36m========== [ REPORT DI PIANIFICAZIONE ] ==========\033[0m\n";
        ss << "  \033[1;33m[PERCEZIONE]\033[0m\n";
        ss << "    - Bordi Mappa:      " << (borders_received_ ? "SI" : "NO") << "\n";
        ss << "    - Poligoni (Box):   " << current_polygons_.size() << "\n";
        ss << "    - Cerchi (Cilindri): " << current_circles_.size() << "\n";
        ss << "  \033[1;35m[VITTIME (" << current_victims_.size() << ")]\033[0m\n";
        for (const auto& v : current_victims_) {
             double d = std::sqrt(std::pow(v.x - start[0], 2) + std::pow(v.y - start[1], 2));
             ss << "    - ID:" << v.id << " (" << v.x << "," << v.y << ") Val:" << (int)v.value << " Dist:" << d << "m\n";
        }
        ss << "  \033[1;33m[QUERY]\033[0m S:(" << start[0] << "," << start[1] << ") -> G:(" << goal[0] << "," << goal[1] << ")\n";
        ss << "  \033[1;33m[DUBINS]\033[0m ";
        if (prim_idx >= 0) ss << getPrimitiveName(prim_idx) << " L=" << curve.L << "m\n";
        else ss << "FAIL\n";
        
        ss << "  \033[1;33m[COLLISIONI]\033[0m ";
        if (safe) ss << "\033[1;32mSICURO\033[0m\n";
        else ss << "\033[1;31mCOLLISIONE\033[0m @ " << collision_s << "m (" << collision_pt[0] << "," << collision_pt[1] << ")\n";
        ss << "\033[1;36m==================================================\033[0m\n";
        ROS_INFO_STREAM_THROTTLE(2.0, ss.str());
    }

    // --- CALLBACK OSTACOLI ---
    void obstaclesCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        current_circles_.clear();
        current_polygons_.clear();
        visualization_msgs::MarkerArray debug_markers;
        int id_counter = 0;

        for (const auto& obs : msg->obstacles) {
            if (obs.polygon.points.size() > 2) { // BOX
                std::vector<Point2D> verts; double sum_x=0, sum_y=0;
                for (const auto& p : obs.polygon.points) { verts.push_back({p.x, p.y}); sum_x+=p.x; sum_y+=p.y; }
                Point2D centroid = {sum_x/verts.size(), sum_y/verts.size()};
                double max_d2 = 0;
                for (const auto& v : verts) max_d2 = std::max(max_d2, std::pow(v.x-centroid.x, 2) + std::pow(v.y-centroid.y, 2));
                
                current_polygons_.push_back({verts, centroid, std::sqrt(max_d2)});

                if (debug_mode_) { // Viz Broad Phase
                    visualization_msgs::Marker broad; broad.header.frame_id="map"; broad.header.stamp=ros::Time::now();
                    broad.ns="broad_phase"; broad.id=id_counter++; broad.type=visualization_msgs::Marker::SPHERE; broad.action=0;
                    broad.pose.position.x=centroid.x; broad.pose.position.y=centroid.y;
                    broad.scale.x=broad.scale.y=broad.scale.z=std::sqrt(max_d2)*2;
                    broad.color.r=1; broad.color.g=1; broad.color.a=0.2;
                    debug_markers.markers.push_back(broad);
                }
            } else if (obs.radius > 0) { // CILINDRI
                Point2D c = {obs.polygon.points[0].x, obs.polygon.points[0].y};
                current_circles_.push_back({c, obs.radius});
                if (debug_mode_) {
                    visualization_msgs::Marker cyl; cyl.header.frame_id="map"; cyl.header.stamp=ros::Time::now();
                    cyl.ns="cylinders"; cyl.id=id_counter++; cyl.type=visualization_msgs::Marker::CYLINDER; cyl.action=0;
                    cyl.pose.position.x=c.x; cyl.pose.position.y=c.y; cyl.pose.position.z=0.5;
                    cyl.scale.x=cyl.scale.y=obs.radius*2; cyl.scale.z=1; cyl.color.b=1; cyl.color.a=0.5;
                    debug_markers.markers.push_back(cyl);
                }
            }
        }
        if (debug_mode_) pub_markers.publish(debug_markers);
        // Ripubblichiamo anche i bordi per sicurezza (se rviz è stato riavviato)
        if (borders_received_ && debug_mode_) publishStaticViz();
    }

    // --- TEST LOOP ---
    void publishTestDubins() {
        // COORDINATE TEST: Per forzare collisione col bordo, facciamolo uscire dalla mappa
        // Assumiamo una mappa 20x20 centrata in 0 (da -10 a 10).
        // Start: (0,0). Goal: (12, 0). Deve collidere col bordo a x=10.
        double start[3] = {0.0, 0.0, 0.0};
        double goal[3]  = {3.0, 2.0, 3.14159}; 
        double r_turn = 1.0; 
        
        int best_prim = -1; dubinscurve_out res;
        dubins_shortest_path(start[0], start[1], start[2], goal[0], goal[1], goal[2], 1.0/r_turn, best_prim, &res);
        if (best_prim < 0) return;

        double col_s = -1.0; double col_pt[2] = {0,0};
        bool safe = checkPathAndVisualize(res, col_s, col_pt);
        printDebugReport(res, best_prim, safe, start, goal, col_s, col_pt);
    }

bool checkPathAndVisualize(const dubinscurve_out& curve, double& out_s, double* out_pt) {
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map"; 
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "dubins_path"; 
        path_marker.id = 999; 
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.scale.x = 0.05; 
        path_marker.color.a = 1.0;
        
        bool collision_found = false;
        double total_s = 0.0;

        auto processArc = [&](const dubinsarc_out& arc) {
            double step = 0.05;
            int n_steps = std::ceil(arc.l / step);
            for(int i=0; i<=n_steps; ++i) {
                double s = (i*step > arc.l) ? arc.l : i*step;
                long double x, y, th;
                circline(s, arc.x0, arc.y0, arc.th0, arc.k, x, y, th);
                
                geometry_msgs::Point p; 
                p.x = (double)x; 
                p.y = (double)y; 
                p.z = 0.0;
                path_marker.points.push_back(p);

                // CHECK COLLISIONE
                // Nota: Usiamo all_polygons che deve essere stato preparato prima, 
                // ma dato che questa funzione è chiamata dentro publishTestDubins,
                // dobbiamo assicurarci di avere accesso ai poligoni.
                // Nel codice precedente usavamo current_circles_ e current_polygons_ membri della classe.
                // Per i bordi statici, dobbiamo fonderli qui o nel chiamante.
                
                // Ricostruiamo la lista completa qui per sicurezza locale
                std::vector<PolygonObstacle> all_polygons = current_polygons_;
                all_polygons.insert(all_polygons.end(), static_borders_.begin(), static_borders_.end());

                if (!collision_found && collision_checker_.check({(double)x, (double)y}, current_circles_, all_polygons)) {
                    collision_found = true;
                    out_s = total_s + s;
                    out_pt[0] = x; 
                    out_pt[1] = y;
                }
            }
            total_s += arc.l;
        };

        processArc(curve.a1);
        processArc(curve.a2);
        processArc(curve.a3);

        path_marker.color.r = collision_found ? 1.0 : 0.0;
        path_marker.color.g = collision_found ? 0.0 : 1.0;
    
        visualization_msgs::MarkerArray ma;
        ma.markers.push_back(path_marker);
        pub_markers.publish(ma);

        return !collision_found;
    }

    void victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        current_victims_.clear(); visualization_msgs::MarkerArray ma; int id=1000;
        for(const auto& o : msg->obstacles) {
            if(o.polygon.points.empty()) continue;
            Victim v = {o.polygon.points[0].x, o.polygon.points[0].y, o.radius, id-1000};
            current_victims_.push_back(v);
            
            visualization_msgs::Marker s; s.header.frame_id="map"; s.header.stamp=ros::Time::now();
            s.ns="victims_viz"; s.id=id++; s.type=2; s.action=0; s.pose.position.x=v.x; s.pose.position.y=v.y;
            s.scale.x=0.5; s.scale.y=0.5; s.scale.z=0.5; s.color.r=1; s.color.a=0.9;
            ma.markers.push_back(s);

            visualization_msgs::Marker t; t.header.frame_id="map"; t.header.stamp=ros::Time::now();
            t.ns="victims_val"; t.id=id++; t.type=9; t.action=0; t.pose.position.x=v.x; t.pose.position.y=v.y; t.pose.position.z=0.6;
            t.text=std::to_string((int)v.value); t.scale.z=0.4; t.color.r=1; t.color.g=1; t.color.b=1; t.color.a=1;
            ma.markers.push_back(t);
        }
        pub_markers.publish(ma);
    }
    void gateCallback(const geometry_msgs::Pose::ConstPtr& msg) {}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_verifier");
    DataVerifier v;
    ros::Rate r(5);
    while(ros::ok()) { v.publishTestDubins(); ros::spinOnce(); r.sleep(); }
    return 0;
}