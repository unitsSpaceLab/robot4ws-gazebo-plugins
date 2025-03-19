

#include <ros/ros.h>
#include <robot4ws_msgs/Vector3Array.h>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <array>
#include <cstring>
#include <thread>
#include <mutex>

namespace gazebo
{

    enum class PluginState {
        UNINITIALIZED,
        LOADING,
        INITIALIZED,
        RUNNING,
        SHUTTING_DOWN
    } plugin_state;    
    
    // Wheel data structure to hold per-wheel information
    struct WheelData {
        std::string name;
        physics::LinkPtr wheel_link;
        physics::LinkPtr steer_link;
        std::string collision_name;
        std::string prev_contact_name = "";
        ignition::math::Quaterniond contact_frame_rot;
        double link_mass;
        double f_signs[2]; // front/back, left/right signs
        
        // Wheel parameters
        struct {
            std::string type;
            double r;   // wheel inner radius, [m]
            double h_g; // grousers height, [m]
            double r_s; // shearing radius, [m]
            double b;   // wheel width, [m]
            double mu;  // grousers area ratio, [-]
        } wheel_params;
        
        // Soil parameters
        struct {
            std::string name;
            double k_c;   // cohesive modulus, [N/m^(n+1)]
            double k_phi; // frictional modulus, [N/m^(n+2)]
            double k;     // sinkage modulus, could be expressed as (k_c/b+k_phi), [N/m^(n+2)]
            double c;     // cohesion, [Pa]
            double phi;   // friction angle, [rad]
            double K;     // shear modulus, [m]
            double rho;   // density, [N/m3]
            double n;     // sinkage exponent, could be expressed in terms of n0, n1, n2, [-]
            double X_c;   // destructive angle, [rad]
        } soil_params;
        
        // Tuned parameters
        struct {
            double a0;  // coeff. for theta_m (tuned), [-]
            double a1;
            double b0;  // coeff. for theta_r (tuned), [-]
            double b1;
            double d0;  // coeff. for theta_0 (tuned), [-]
            double d1;
            double n0;  // coeff. for sinkage exponent n (tuned), [-]
            double n1;
            double n2;
        } tuned_params;
        
        // Wheel state parameters
        struct {
            double v_x;     // wheel longitudinal velocity, [m/s]
            double v_y;     // wheel lateral velocity, [m/s]
            double v;       // wheel velocity, [m/s]
            double omega;   // wheel angular velocity, [rad/s]
            double s;       // slip/skid ratio, [-]
            double beta;    // slip angle, [rad]
        } wheel_state_params;
        
        // Terramechanics parameters
        struct {
            double h_0;                 // wheel sinkage, [m]
            std::vector<double> h;      // point depht (referred to r_s), [m]
            double theta_f;             // entry angle, [rad]
            double theta_r;             // exit angle, [rad]
            double theta_m;             // maximum normal stress angle, [rad]
            std::vector<double> theta;  // point coordinate, [rad]
            std::vector<double> theta_e;// equivalent front angle for rear region points, [rad]
            double theta_0;             // shear transition angle in skid, [rad]
            std::vector<double> sigma;  // normal stress, [Pa]
            std::vector<double> tau;    // shear stress, [Pa]
            std::vector<double> tau_t;  // tangential shear stress, [Pa]
            std::vector<double> tau_l;  // lateral shear stress, [Pa]
            std::vector<double> j;      // shear displacement, [m]
            std::vector<double> j_t;    // tangential shear displacement, [m]
            std::vector<double> j_l;    // lateral shear displacement, [m]
            std::vector<double> v_jt;   // tangential shear velocity, [m/s]
            std::vector<double> v_jl;   // lateral shear velocity, [m/s]
        } terramechanics_params;
        
        // Forces
        struct {
            double W;                 // wheel vertical load (constant), [N]
            std::vector<double> R_b;  // bulldozing resistence, [Pa*m]
            double F_x;               // force acting on the wheel along X axis, [N]
            double F_y;               // force acting on the wheel along Y axis, [N]
            double F_z;               // force acting on the wheel along Z axis, [N]
            double M_x;               // [N*m]
            double M_y;               // driving moment (around Y axis), [N*m]
            double M_z;               // [N*m]
        } forces;
        
        // ROS publisher and message
        ros::Publisher ros_pub_results;
        robot4ws_msgs::Vector3Array resultMsg;
        
        // Initialize vectors
        void initVectors(int rim_points) {
            terramechanics_params.h.resize(rim_points, 0.0);
            terramechanics_params.theta.resize(rim_points, 0.0);
            terramechanics_params.theta_e.resize(rim_points, 0.0);
            terramechanics_params.sigma.resize(rim_points, 0.0);
            terramechanics_params.tau.resize(rim_points, 0.0);
            terramechanics_params.tau_t.resize(rim_points, 0.0);
            terramechanics_params.tau_l.resize(rim_points, 0.0);
            terramechanics_params.j.resize(rim_points, 0.0);
            terramechanics_params.j_t.resize(rim_points, 0.0);
            terramechanics_params.j_l.resize(rim_points, 0.0);
            terramechanics_params.v_jt.resize(rim_points, 0.0);
            terramechanics_params.v_jl.resize(rim_points, 0.0);
            forces.R_b.resize(rim_points, 0.0);
        }
    };

    class ApplyAllWheelsTerramechanicModel : public ModelPlugin
    {
        public:
            ApplyAllWheelsTerramechanicModel(); // Constructor
            virtual ~ApplyAllWheelsTerramechanicModel(); // Destructor
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
            virtual void Init() override;
            virtual void Reset() override;
            void OnUpdate();
            void OnUpdateCompact();

        private: 
            // Constants
            static constexpr int num_wheels = 4;
            static constexpr int rim_points = 100;
            
            // Wheel names and indices
            const std::array<std::string, num_wheels> wheel_names = {
                "Archimede_br_wheel_link",
                "Archimede_fr_wheel_link", 
                "Archimede_bl_wheel_link", 
                "Archimede_fl_wheel_link"
            };
            
            // Plugin options
            struct {
                int rim_pts = rim_points;
                std::string bulldozing_resistence = "neglect";
                bool use_compact_model;
            } options;
            
            // Initialization methods
            void initializePluginParam();
            void initializeROSelements();
            void initializeWheels();
            bool initializeWheel(int wheel_idx);
            
            // Wheel processing methods
            void processWheel(int wheel_idx);
            void processWheelCompact(int wheel_idx);
            
            // Per-wheel calculation methods
            bool checkContacts(int wheel_idx);
            bool findSinkage(int wheel_idx);
            void computeContactGeometry(int wheel_idx);
            void computeStresses(int wheel_idx);
            void computeForces(int wheel_idx, double* F_z_out = nullptr);
            void computeWheelLoad(int wheel_idx);
            void setWheelParams(int wheel_idx);
            void setSoilParams(int wheel_idx);
            void setWheelStateParams(int wheel_idx);
            void setTunedParams(int wheel_idx);
            void applyForce(int wheel_idx);
            void publishResults(int wheel_idx);
            
            // Common data
            std::array<WheelData, num_wheels> wheels;
            sdf::ElementPtr sdf;
            physics::ModelPtr model;
            physics::ContactManager* contact_manager;
            double rover_dimensions[2];
            std::vector<std::vector<double>> other_masses;
            double world_gravity;
            
            // Settings
            bool debug;
            bool publish_results;
            
            // ROS elements
            ros::NodeHandle* _ros_node;
            std::string ros_pub_results_topic_name;
            
            // Gazebo event connection
            event::ConnectionPtr updateConnection;
            
            // Contact handling
            transport::NodePtr dummy_contact_node;
            transport::SubscriberPtr dummy_contact_sub;
            void dummy_contact_callback(ConstWorldStatisticsPtr &_msg) { return; }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ApplyAllWheelsTerramechanicModel)
}
