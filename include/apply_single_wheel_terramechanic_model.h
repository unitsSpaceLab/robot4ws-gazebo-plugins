/**
* @author Simone Cottiga
* @email: simone.cottiga@phd.units.it
* @email: smcotti@gmail.com
**/

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


// CHECK message type
// #define SLIP_MESSAGE_TYPE robot4ws_msgs::Vector3Array

namespace gazebo
{
    class ApplySingleWheelTerramechanicModel : public ModelPlugin
    {
        public:
            ApplySingleWheelTerramechanicModel(); //Constructor
            virtual ~ApplySingleWheelTerramechanicModel(); //Destructor
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            virtual void OnUpdate(void);
            virtual void OnUpdateCompact(void);

        private: 
            void initializePluginParam(void);
            void initializeLinks(void);
            void initializeROSelements(void);

            bool checkContacts(void);
            bool findSinkage(void);
            void computeContactGeometry(void);
            void computeStresses(void);
            void computeForces(double* F_z_out = nullptr);
            void computeWheelLoad(void);

            void setWheelParams(void);
            void setSoilParams(void);
            void setWheelStateParams(void);
            void setTunedParams(void);

            void applyForce(void);
            void publishResults(void);

            static constexpr int rim_points = 100;

            struct {
                int rim_pts = rim_points;    // rim discretization points
                std::string bulldozing_resistence = "Ishigami";  // model for sidewall bulldozing force:...
                                                // "Ishigami", "Pavlov" (for Pavlov/Wong) or "neglect"
                bool use_compact_model; // use compact model for compute forces/moments instead of integrating them along the wheel rim
            } options;

            struct {
                std::string type;
                double r;   // wheel inner radius (at the base of the lugs), [m]
                double h_g; // grousers height, [m]
                double r_s; // shearing radius, [m]
                double b;   // wheel width, [m]
                double mu;  // grousers area ratio, [-]
            } wheel_params;

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

            struct {
                double v_x;     // wheel longitudinal velocity, [m/s]
                double v_y;     // wheel lateral velocity, [m/s]
                double v;       // wheel velocity, [m/s]
                double omega;   // wheel angular velocity, [rad/s]
                double s;       // slip/skid ratio, [-]
                double beta;    // slip angle, [rad]
            } wheel_state_params;

            struct {
                double h_0;                 // wheel sinkage, [m]
                double h[rim_points];       // point depht (referred to r_s), [m]
                double theta_f;             // entry angle, [rad]
                double theta_r;             // exit angle, [rad]
                double theta_m;             // maximum normal stress angle, [rad]
                double theta[rim_points];   // point coordinate, [rad]
                double theta_e[rim_points]; // equivalent front angle for rear region points, [rad]
                double theta_0;             // shear transition angle in skid, [rad]
                double sigma[rim_points];   // normal stress, [Pa]
                double tau[rim_points];     // shear stress, [Pa]
                double tau_t[rim_points];   // tangential shear stress, [Pa]
                double tau_l[rim_points];   // lateral shear stress, [Pa]
                double j[rim_points];       // shear displacement, [m]
                double j_t[rim_points];     // tangential shear displacement, [m]
                double j_l[rim_points];     // lateral shear displacement, [m]
                double v_jt[rim_points];    // tangential shear velocity, [m/s]
                double v_jl[rim_points];    // lateral shear velocity, [m/s]
            } terramechanics_params;

            struct {
                double W;               // wheel vertical load (constant), [N]
                double R_b[rim_points]; // bulldozing resistence, [Pa*m]
                double F_x;             // force acting on the wheel along X axis, [N]
                double F_y;             // force acting on the wheel along Y axis, [N]
                double F_z;             // force acting on the wheel along Z axis, [N]
                double M_x;             // [N*m]
                double M_y;             // driving moment (around Y axis), [N*m]
                double M_z;             // [N*m]
            } forces;

            sdf::ElementPtr sdf;
            physics::ModelPtr model;
            physics::LinkPtr link_wheel;
            physics::LinkPtr link_steer;

            ignition::math::Quaterniond contact_frame_rot; // rotation of contact frame, defined as: z axis along contact normal, x is the projection of steer x axis (wheel radial dir) pointing forward, y 3rd with right hand rule
            ignition::math::Quaterniond steer_2_wheel_orient;

            physics::ContactManager * contact_manager; // pointer to the contact manager

            std::string link_name;              // wheel link
            std::string link_collision_name;    // wheel link
            std::string prev_contact_name = "";

            bool debug;

            bool publish_results;
            ros::NodeHandle* _ros_node;
            ros::Publisher ros_pub_results;
            std::string ros_pub_results_topic_name;
            robot4ws_msgs::Vector3Array resultMsg;

            double link_mass;   // wheel + steer, this is applied in the CoG of the link
            double rover_dimensions[2]; // longitudinal & lateral distances between the wheels
            std::vector<std::vector<double>> other_masses;    // [mass, long, lat, elev] of each link that is not a wheel or steer. long, lat, elev are 
                                                                // the 3 components of distance |wheel_position - mass_position| (constant)
            double f_signs[2];  // front: 1st=1, back: 1st=-1, left: 2nd=1, right: 2nd=-1
            double world_gravity; // assumed direction [0,0,-1] in world frame

            event::ConnectionPtr updateConnection; // Pointer to the update event connection

            // START this part might be needed in order to use the gazebo contacts, in case...
            // there is nothing else listening to them (ex. contact sensors or client->View->Contacts activated)
            transport::NodePtr dummy_contact_node;
            transport::SubscriberPtr dummy_contact_sub;
            inline void dummy_contact_callback(ConstWorldStatisticsPtr &_msg){return;}
            // END this part might be needed in order to use the gazebo contacts, in case...
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ApplySingleWheelTerramechanicModel)
}