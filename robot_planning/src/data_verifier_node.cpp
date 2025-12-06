#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cmath>
#include <algorithm>

// --- HEADER PROGETTO ---
#include "dubins_planner/dubins_trajectory.h"
#include "dubins_planner/collision_checker.h" 

// ============================================================================
// DEFINIZIONE VARIABILI GLOBALI (Richieste dalla libreria Dubins C-Style)
// ============================================================================
bool DEBUG = false;
long double X0, Y0, Th0, Xf, Yf, Thf, Kmax;
int pidx;
int no_waypts, step, no_of_samples;
long double angle_step;
dubinscurve_out dubin_curve;
point init_pt, final_pt;
std::vector<point> best_path;
// ============================================================================

class DataVerifier {
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_obs;
    ros::Subscriber sub_victims;
    ros::Subscriber sub_gate;
    ros::Publisher pub_markers;

    bool gate_received = false;

    // --- COLLISION CHECKER ---
    CollisionChecker collision_checker_;
    
    // Cache degli ostacoli convertiti per il checker
    std::vector<CircleObstacle> current_circles_;
    std::vector<PolygonObstacle> current_polygons_;

public:
    // Costruttore: Imposta Raggio Robot (0.25m) e Margine (0.05m) -> Totale 0.3m
    DataVerifier() : collision_checker_(0.25, 0.05) {
        pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/debug/environment_markers", 1, true);
        
        sub_obs = nh.subscribe("/obstacles", 1, &DataVerifier::obstaclesCallback, this);
        sub_victims = nh.subscribe("/victims", 1, &DataVerifier::victimsCallback, this);
        sub_gate = nh.subscribe("/gate_position", 1, &DataVerifier::gateCallback, this);
        
        ROS_INFO("--- DATA VERIFIER NODE STARTED ---");
        ROS_INFO("Collision Checker Initialized (Robot Radius: 0.25m, Margin: 0.05m)");
    }

    // --- CALLBACK OSTACOLI (Conversione Dati per Collision Checker) ---
    void obstaclesCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        current_circles_.clear();
        current_polygons_.clear();

        for (const auto& obs : msg->obstacles) {
            // Caso 1: Poligono (se ha più di 2 punti)
            if (obs.polygon.points.size() > 2) {
                std::vector<Point2D> verts;
                double sum_x = 0, sum_y = 0;
                
                // Estrai vertici e calcola somma per centroide
                for (const auto& p : obs.polygon.points) {
                    verts.push_back({p.x, p.y});
                    sum_x += p.x;
                    sum_y += p.y;
                }
                
                // 1. Calcola Centroide
                Point2D centroid = {sum_x / verts.size(), sum_y / verts.size()};

                // 2. Calcola Bounding Radius (distanza massima dal centroide)
                double max_dist_sq = 0;
                for (const auto& v : verts) {
                    double dx = v.x - centroid.x;
                    double dy = v.y - centroid.y;
                    max_dist_sq = std::max(max_dist_sq, dx*dx + dy*dy);
                }

                // 3. Aggiungi alla lista
                current_polygons_.push_back({verts, centroid, std::sqrt(max_dist_sq)});
            }
            // Caso 2: Cerchio (se ha raggio > 0 e almeno un punto per il centro)
            else if (obs.radius > 0 && !obs.polygon.points.empty()) {
                Point2D center = {obs.polygon.points[0].x, obs.polygon.points[0].y};
                current_circles_.push_back({center, obs.radius});
            }
        }
    }

    // --- CHECKER SICUREZZA PERCORSO ---
    bool isPathSafe(const dubinscurve_out& curve) {
        if (!checkArcSafety(curve.a1)) return false;
        if (!checkArcSafety(curve.a2)) return false;
        if (!checkArcSafety(curve.a3)) return false;
        return true;
    }

    bool checkArcSafety(const dubinsarc_out& arc) {
        double step_size = 0.05; // Check ogni 5cm
        int steps = std::ceil(arc.l / step_size);
        
        for (int i = 0; i <= steps; ++i) {
            double s = i * step_size;
            if (s > arc.l) s = arc.l; 

            long double x, y, th;
            circline(s, arc.x0, arc.y0, arc.th0, arc.k, x, y, th);

            // Verifica collisione puntuale
            if (collision_checker_.check({(double)x, (double)y}, current_circles_, current_polygons_)) {
                return false; // Collisione!
            }
        }
        return true;
    }

    // --- VISUALIZZAZIONE E TEST ---
    void publishTestDubins() {
        // 1. Configurazione Test: Genera un percorso che attraversa l'origine (spesso pericoloso)
        double start_x = 0.0, start_y = 0.0, start_th = 0.0;
        double goal_x = 4.0, goal_y = 2.0, goal_th = 1.57; // Target ipotetico
        
        double turning_radius = 0.5;
        double k_max = 1.0 / turning_radius;

        // 2. Calcolo Percorso Dubins
        int best_primitive_idx = -1;
        dubinscurve_out result_curve;
        
        dubins_shortest_path(start_x, start_y, start_th, 
                             goal_x, goal_y, goal_th, 
                             k_max, best_primitive_idx, &result_curve);

        if (best_primitive_idx < 0) return;

        // 3. Verifica Sicurezza
        bool safe = isPathSafe(result_curve);

        // 4. Visualizzazione Marker
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "dubins_check";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.scale.x = 0.05; 
        
        // Logica Colore: VERDE = OK, ROSSO = COLLISIONE
        if (safe) {
            path_marker.color.r = 0.0; path_marker.color.g = 1.0; path_marker.color.b = 0.0;
        } else {
            path_marker.color.r = 1.0; path_marker.color.g = 0.0; path_marker.color.b = 0.0;
            ROS_WARN_THROTTLE(2.0, "COLLISIONE RILEVATA SUL PERCORSO DUBINS!");
        }
        path_marker.color.a = 1.0;
        path_marker.lifetime = ros::Duration(0);

        sampleAndAddToMarker(&result_curve.a1, path_marker);
        sampleAndAddToMarker(&result_curve.a2, path_marker);
        sampleAndAddToMarker(&result_curve.a3, path_marker);

        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(path_marker);
        pub_markers.publish(marker_array);
    }

    void sampleAndAddToMarker(dubinsarc_out* arc, visualization_msgs::Marker& marker) {
        double step_size = 0.1; 
        int steps = std::ceil(arc->l / step_size);
        for (int i = 0; i <= steps; ++i) {
            double s = i * step_size;
            if (s > arc->l) s = arc->l;
            long double x, y, th;
            circline(s, arc->x0, arc->y0, arc->th0, arc->k, x, y, th);
            geometry_msgs::Point p;
            p.x = (double)x; p.y = (double)y; p.z = 0.0;
            marker.points.push_back(p);
        }
    }

    // --- CALLBACK AUSILIARIE (Vittime/Gate) ---
    void victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        if (msg->obstacles.empty()) return;
        visualization_msgs::MarkerArray marker_array;
        int id = 1000;
        for (const auto& victim : msg->obstacles) {
            if (victim.polygon.points.empty()) continue;
            visualization_msgs::Marker sphere;
            sphere.header.frame_id = "map";
            sphere.header.stamp = ros::Time::now();
            sphere.ns = "victims";
            sphere.id = id++;
            sphere.type = visualization_msgs::Marker::SPHERE;
            sphere.action = visualization_msgs::Marker::ADD;
            sphere.pose.position.x = victim.polygon.points[0].x; 
            sphere.pose.position.y = victim.polygon.points[0].y;
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = 0.5; sphere.scale.y = 0.5; sphere.scale.z = 0.1;
            sphere.color.r = 1.0; sphere.color.a = 0.8;
            marker_array.markers.push_back(sphere);
        }
        pub_markers.publish(marker_array);
    }

    void gateCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        if (gate_received) return;
        gate_received = true;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "gate";
        marker.id = 9999;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = *msg;
        marker.scale.x = 1.0; marker.scale.y = 0.2; marker.scale.z = 3.0;
        marker.color.b = 1.0; marker.color.a = 1.0;
        visualization_msgs::MarkerArray ma; ma.markers.push_back(marker);
        pub_markers.publish(ma);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_verifier");
    DataVerifier verifier;
    
    ros::Rate r(10); // 10 Hz
    while(ros::ok()) {
        verifier.publishTestDubins();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}