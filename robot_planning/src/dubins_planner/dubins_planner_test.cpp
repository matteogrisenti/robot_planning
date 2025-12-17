#include <ros/ros.h>
#include <robot_control/Reference.h> // To verify the output messages
#include <visualization_msgs/MarkerArray.h>

// Include the Planner Class we want to test
#include "dubins_planner/dubins_planner.h"
#include "map/map_builder.h"

// --- LEGACY GLOBAL VARIABLES (Required for linking dubins_trajectory library) ---
bool DEBUG = false;
long double X0, Y0, Th0, Xf, Yf, Thf, Kmax;
int pidx;
int no_waypts, step, no_of_samples;
long double angle_step;
dubinscurve_out dubin_curve;
point init_pt, final_pt;
std::vector<point> best_path;
// --------------------------------------------------------------------------------

class DubinsPlannerTester {
private:
    ros::NodeHandle nh_;
    
    // The Object Under Test (OUT)
    DubinsPlanner planner_; 
    
    // Verification Subscriber (listens to the output of the planner)
    ros::Subscriber ref_verification_sub_;
    
    Point goal_pos;
    double goal_theta = 0.0;
    bool test_running_ = false;

public:
    DubinsPlannerTester() 
        : nh_("~"), 
          // Initialize Planner with specific robot namespace
          planner_(nh_, "limo0", 0.25, 0.05) 
    {
        // 1. Setup Verification: Subscribe to the topic the planner SHOULD publish to
        ref_verification_sub_ = nh_.subscribe("/limo0/ref", 1, 
                                              &DubinsPlannerTester::verificationCallback, this);
        
        // 2. Setup Environment (Map & Goal)
        setupEnvironment();
        
        // 3. Run the Test Sequence
        runTestSequence();
    }

    void setupEnvironment() {
        ROS_INFO("[Tester] 1. Building Map...");
        map_builder::MapBuilder builder(nh_, 100.0);
        Map map = builder.buildMap();
        
        // INJECT MAP into the planner
        planner_.setMap(map);
        
        // Extract Goal from the first gate found
        const auto& all_gates = map.gates.get_gates();
        if (!all_gates.empty()) {
            const Point& p = all_gates[0].get_position();
            goal_pos = p;
            
            // Calc orientation from quaternion (assuming simple yaw)
            const Orientation& o = all_gates[0].get_orientation();
            goal_theta = std::atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z));
            
            ROS_INFO("[Tester] Goal Set from Map: [%.2f, %.2f, %.2f]", goal_pos.x, goal_pos.y, goal_theta);
        } else {
            ROS_WARN("[Tester] No gates found! Using default goal (2.0, 0.0, 0.0)");
            goal_pos = {2.0, 0.0, 0.0};
            goal_theta = 0.0;
        }
    }

    void runTestSequence() {
        ROS_INFO("[Tester] 2. Testing PLANNER Logic...");
        
        // Define Start Pose (Test Scenario)
        double start_x = 0.0;
        double start_y = 0.0;
        double start_th = 0.0;
        double rho = 0.5; // Min turning radius

        // CALL PLANNER: This should trigger internal collision checks and Rviz visualization
        bool success = planner_.planPath(start_x, start_y, start_th, 
                                         goal_pos.x, goal_pos.y, goal_theta, 
                                         rho, true); // Enable debug viz

        if (success) {
            ROS_INFO("[Tester] Plan Valid! (Check Rviz for Green Path)");
            ROS_INFO("[Tester] 3. Testing EXECUTION Logic...");
            
            // Start Execution (Simulating the robot moving)
            planner_.startExecution(0.5); // Target velocity 0.5 m/s
            test_running_ = true;
        } else {
            ROS_ERROR("[Tester] Plan Failed (Collision or Geometric). Check Rviz for Red Path.");
        }
    }

    // This callback mimics the controller receiving messages
    void verificationCallback(const robot_control::Reference::ConstPtr& msg) {
        if (msg->plan_finished) {
            ROS_INFO("[Tester] VERIFIED: Received 'Plan Finished' signal.");
            test_running_ = false;
        } else {
            // Log every ~1 second to confirm data flow
            ROS_INFO_THROTTLE(1.0, "[Tester] VERIFIED: Topic '/limo0/ref' active. v=%.2f, w=%.2f, x=%.2f", 
                              msg->v_d, msg->omega_d, msg->x_d);
        }
    }

    void spin() {
        if (test_running_) {
            // Drive the planner execution loop
            planner_.spin();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dubins_planner_tester");
    
    DubinsPlannerTester tester;
    
    ros::Rate r(50); // 50 Hz simulation loop
    while(ros::ok()) {
        tester.spin();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}