/**
* @author Simone Cottiga
* @email: simone.cottiga@phd.units.it
* @email: smcotti@gmail.com
**/

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot4ws_msgs/Vector3Array.h>
#include <robot4ws_msgs/JointState1.h>
#include <geometry_msgs/PoseArray.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include <string>
#include <fstream>
#include <iostream>

// CHECK message type
#define SLIP_MESSAGE_TYPE robot4ws_msgs::Vector3Array

namespace gazebo
{
    class ArchimedeApplyArtificialSlip : public ModelPlugin
    {
        public:
            ArchimedeApplyArtificialSlip(); //Constructor
            virtual ~ArchimedeApplyArtificialSlip(); //Destructor
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
            virtual void OnUpdate(void);

        private: 
            void initializePluginParam(void);
            void initializeLinks(void);
            void initializeForcePID(void);
            void resetForcePID(void);
            void initializeROSelements(void);

            template <class joint_states_message_type>
            void jointStateCallback(const joint_states_message_type &joint_msg);
            void slipVelCallback(const SLIP_MESSAGE_TYPE::ConstPtr &vel_msg);
            void publishResults(void);
            void printResults(void);
            void applyForce(void);
            void applyVelocity(void);
            void updateTargetVelocity(void);
            void publishPluginValidation(void);


            int link_map[8];
            double joint_velocities[8];
            double wheel_radius = 0.085;
            bool is_simulation; // true for gazebo simulation, false for physical rover. (just for joint_states topic and link mapping)

            sdf::ElementPtr sdf;
            physics::ModelPtr model; // Pointer to the model
            physics::LinkPtr link[4];

            std::string apply_mode; // apply force or velocity
            std::vector<std::string> link_name;

            ignition::math::Vector3d theor_vel[4];     // theoretical wheel velocity in world frame calculated from wheel angular velocity
            ignition::math::Vector3d real_vel[4];      // real velocity in world frame, only used in force mode to set the pid error
            ignition::math::Vector3d target_slip[4];   // velocity that must be added to the wheel, in world frame
            ignition::math::Vector3d target_vel[4];    // target velocity (theor_vel + target_slip), in world frame

            common::PID pid_X_force[4];
            common::PID pid_Y_force[4];
            common::PID pid_Z_force[4];

            common::Time time_now;
            common::Time time_prev;

            ros::NodeHandle* _ros_node;
            std::string joint_states_topic_name; // topic which we get the joint states from (commanded velocities)
            ros::Subscriber _joint_states_sub;
            std::string slip_velocities_topic_name;  // topic which we get slip velocities from, either as drift component or directly as target velocities to apply
            ros::Subscriber _slip_velocities_sub;
            bool is_target_vel_pub; // true: the _velocity_sub topic publishes the target velocities
                                    // false: it publishes the drift velocitites (target will be drift+commanded)

            bool check_results;
            bool print_results;
            ros::Publisher pub_results;
            std::string pub_results_topic_name;
            robot4ws_msgs::Vector3Array resultMsg;

            bool validate_plugin; // plugin validation, publish steers poses
            ros::Publisher valid_plug_pub;
            std::string valid_plug_topic_name;
            geometry_msgs::PoseArray valid_plug_msg;

            event::ConnectionPtr updateConnection; // Pointer to the update event connection
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ArchimedeApplyArtificialSlip)
}