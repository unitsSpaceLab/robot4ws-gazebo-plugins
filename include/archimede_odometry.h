

#ifndef ARCHIMEDE_ODOMETRY_H
#define ARCHIMEDE_ODOMETRY_H


#include "constants.h"
#include "utility.h"


// Include ROS related stuff
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>


// Include Rover related custom messages
#include <robot4ws_msgs/Motor_current_int.h>
#include <robot4ws_msgs/Motor_temperature_int.h>
#include <robot4ws_msgs/Operating_modes_int.h>
#include <robot4ws_msgs/Sound.h>
#include <robot4ws_msgs/JointState1.h>
#include <robot4ws_msgs/SensorState.h>


// Include Gazebo related stuff
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>
#include <gazebo/physics/physics.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>




// Include other useful libraries
#include<vector>
#include<stdio.h>
#include<string>
#include<map>
#include<cmath>


#define MOTORS 8





namespace gazebo
{
    class ArchimedeOdometryPlugin : public ModelPlugin
    {
        public:
            ArchimedeOdometryPlugin();
            virtual ~ArchimedeOdometryPlugin();

            void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
            virtual bool OnUpdate(void);
            void Reset(void);

            void publishOdom(void);




        private:
            void initializeROSelements(void);
            std::vector<double> getWheelsState(void);
            std::vector<double> getSteersState(void);
            void initializeOdometry(void);
            std::pair<std::vector<double>, std::vector<double>> readEncoders(void);
            void publishTF(void);
            void publishRealTF(void);

            void printJointsInfo(void);

            void _parseSDFparams(void);


            std::string _integrator;
            std::string _odom_mode;

            event::ConnectionPtr connection;

            bool print_debug = false;
            bool print_mapping_debug = true;
            bool print_joints_info = false;
            bool debug_matrices = false;
            bool print_wheel_readings = false;
            bool print_debug_update = false;


            bool publish_body_real_frame = true;


            std::string _odom_frame_name;

            physics::ModelPtr _model;
            sdf::ElementPtr _sdf; 
            physics::WorldPtr _world;


            //Define ROS related stuff 
            ros::NodeHandle* _nh;
            ros::Publisher _odom_pub;
            ros::Publisher _joint_state_pub;


            //Define joints container
            std::vector<physics::JointPtr> _joints;


            //Define joints mapping
            std::map<int, joint_pair> _joint_mapping;

            //Odom plugin config
            odom_config _odom_config;
            std::vector<ros::Publisher> _fake_pub_list;


            //Time stuff
            common::Time last_update_time;


            //Last motors readings
            std::pair<std::vector<double>, std::vector<double>> _last_readings;


            //Pose stuffs
            ignition::math::Vector3<double> last_pose;
            double last_theta;
            double publis_rate;
            common::Time last_time_published;

            //Matrices definition for the odometry
            std::vector<std::vector<double> > B_Matrix;
            std::vector<std::vector<double> > A_Matrix;
            std::vector<std::vector<double> > P_inv_A;

            //Define odometry message
            nav_msgs::Odometry _odom_msg;

            //Define transform broadcaster
            tf::TransformBroadcaster _tf;


            //Last delta wheels displacement
            double _last_delta_s_forward[MOTORS/2] = {0,};

            int _skip_frames;
            int _step;



    };
    GZ_REGISTER_MODEL_PLUGIN(ArchimedeOdometryPlugin)


}

#endif //ARCHIMEDE_ODOMETRY_H