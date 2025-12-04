#include <ros/ros.h>
#include <obstacles_msgs/ObstacleArrayMsg.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

// --- INTEGRAZIONE LIBRERIA DUBINS ---
#include "dubins_planner/dubins_trajectory.h"

// ============================================================================
// DEFINIZIONE VARIABILI GLOBALI (Richieste dalla libreria "sporca" C-Style)
// Senza queste righe, avrai errori di linker "undefined reference"
// ============================================================================
bool DEBUG = false; // Mettilo a true se vuoi spam nel terminale
long double X0, Y0, Th0, Xf, Yf, Thf, Kmax;
int pidx;
int no_waypts, step, no_of_samples;
long double angle_step;
dubinscurve_out dubin_curve;
point init_pt, final_pt; // Rinominato per evitare conflitti con init()
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

public:
    DataVerifier() {
        pub_markers = nh.advertise<visualization_msgs::MarkerArray>("/debug/environment_markers", 1, true);
        sub_obs = nh.subscribe("/obstacles", 1, &DataVerifier::obstaclesCallback, this);
        sub_victims = nh.subscribe("/victims", 1, &DataVerifier::victimsCallback, this);
        sub_gate = nh.subscribe("/gate_position", 1, &DataVerifier::gateCallback, this);
        
        ROS_INFO("--- DATA VERIFIER + DUBINS TESTER STARTED ---");
    }

    // --- FUNZIONI DI VISUALIZZAZIONE ESISTENTI (Vittime/Gate) ---
    void victimsCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        if (msg->obstacles.empty()) return;
        visualization_msgs::MarkerArray marker_array;
        int id = 1000;
        double max_score = 1.0;
        for (const auto& victim : msg->obstacles) if (victim.radius > max_score) max_score = victim.radius;

        for (const auto& victim : msg->obstacles) {
            if (victim.polygon.points.empty()) continue;
            double cx = victim.polygon.points[0].x;
            double cy = victim.polygon.points[0].y;
            double score = victim.radius;
            double scale_factor = std::max((score / max_score) * 1.0, 0.2);

            visualization_msgs::Marker sphere;
            sphere.header.frame_id = "map";
            sphere.header.stamp = ros::Time::now();
            sphere.ns = "victims";
            sphere.id = id++;
            sphere.type = visualization_msgs::Marker::SPHERE;
            sphere.action = visualization_msgs::Marker::ADD;
            sphere.pose.position.x = cx; sphere.pose.position.y = cy; sphere.pose.position.z = 0.0;
            sphere.pose.orientation.w = 1.0;
            sphere.scale.x = scale_factor; sphere.scale.y = scale_factor; sphere.scale.z = 0.1;
            sphere.color.g = (float)(score / max_score); sphere.color.a = 0.8; sphere.color.r = 0.0; sphere.color.b = 0.0;
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
        marker.pose.position.z = 1.0;
        marker.scale.x = 1.0; marker.scale.y = 0.2; marker.scale.z = 5.0;
        marker.color.b = 1.0; marker.color.a = 1.0;
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        pub_markers.publish(marker_array);
    }

    void obstaclesCallback(const obstacles_msgs::ObstacleArrayMsg::ConstPtr& msg) {
        // Implementazione opzionale per ostacoli
    }

    // --- NUOVA LOGICA DUBINS ---

    // Helper per campionare un arco usando la funzione 'circline' della libreria
    void sampleAndAddToMarker(dubinsarc_out* arc, visualization_msgs::Marker& marker) {
        double step_size = 0.1; // Campionamento ogni 10cm
        int steps = std::ceil(arc->l / step_size);
        
        for (int i = 0; i <= steps; ++i) {
            double s = i * step_size;
            if (s > arc->l) s = arc->l; // Clamp

            long double x, y, th; // Tipi della libreria
            // Chiamata alla funzione della libreria importata
            circline(s, arc->x0, arc->y0, arc->th0, arc->k, x, y, th);

            geometry_msgs::Point p;
            p.x = (double)x;
            p.y = (double)y;
            p.z = 0.0;
            marker.points.push_back(p);
        }
    }

    void publishTestDubins() {
        // 1. Setup Parametri (Usiamo le variabili globali della libreria o parametri locali passati alla funzione)
        double start_x = 0.0, start_y = 0.0, start_th = 0.0;
        double goal_x = -3.0, goal_y = 2.0, goal_th = 3.14; // Una manovra "indietro" complessa
        
        // Raggio minimo 0.5m -> Curvatura Max = 1/0.5 = 2.0
        double turning_radius = 0.5;
        double k_max = 1.0 / turning_radius;

        // 2. Calcolo (Chiamata alla funzione C-style)
        int best_primitive_idx = -1;
        dubinscurve_out result_curve;
        
        // Firma: (x0, y0, th0, xf, yf, thf, Kmax, &pidx, &curve)
        dubins_shortest_path(start_x, start_y, start_th, 
                             goal_x, goal_y, goal_th, 
                             k_max, best_primitive_idx, &result_curve);

        
        // Debug: Prima controlla se il calcolo è andato a buon fine
        if (best_primitive_idx < 0) {
            ROS_ERROR_THROTTLE(5, "Dubins: No valid path found! pidx=%d", best_primitive_idx);
            return;
        }
        
        ROS_INFO_THROTTLE(5, "Dubins: pidx=%d, Length=%.2f, a1.l=%.2f, a2.l=%.2f, a3.l=%.2f", 
                         best_primitive_idx, (double)result_curve.L,
                         (double)result_curve.a1.l, (double)result_curve.a2.l, (double)result_curve.a3.l);
        
        // 3. Visualizzazione
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "dubins_test_path";
        path_marker.id = 666;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.scale.x = 0.05; // Spessore linea
        path_marker.color.r = 0.0; path_marker.color.g = 0.5; path_marker.color.b = 1.0; 
        path_marker.color.a = 1.0;
        path_marker.lifetime = ros::Duration(0);

        // Campioniamo i 3 segmenti (LSL, RSR, etc sono composti da 3 archi: a1, a2, a3)
        sampleAndAddToMarker(&result_curve.a1, path_marker);
        sampleAndAddToMarker(&result_curve.a2, path_marker);
        sampleAndAddToMarker(&result_curve.a3, path_marker);

        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(path_marker);
        pub_markers.publish(marker_array);
        
        // Debug Log (una volta ogni tanto)
        ROS_INFO_THROTTLE(5, "Dubins Path Calc. Length: %.2f", (double)result_curve.L);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_verifier");
    DataVerifier verifier;
    
    // Loop principale
    ros::Rate r(2); // 2 Hz
    while(ros::ok()) {
        verifier.publishTestDubins(); // Esegue il test continuamente
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}