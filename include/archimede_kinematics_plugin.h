#ifndef ARCHIMEDE_KINEMATICS_PLUGIN_H
#define ARCHIMEDE_KINEMATICS_PLUGIN_H


// ROS dependencies
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <robot4ws_msgs/Dynamixel_parameters1.h>


// Custom Callback que
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>



// Boost stuff
#include <boost/thread.hpp>
#include <boost/bind.hpp>



// Gazebo dependecies
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo/physics/physics.hh>


// Other stuff
#include <string>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <map>



#define MOTORS 8





namespace gazebo
{
    class ArchimedeKinematicsPlugin : public ModelPlugin
    {
        public:
            ArchimedeKinematicsPlugin(); //Constructor     (WRITTEN)
            virtual ~ArchimedeKinematicsPlugin(); //Destructor     (WRITTEN)
            void Load(physics::ModelPtr model, sdf::ElementPtr sdf); //     (WRITTEN)
            virtual void OnUpdate(void); //Callback called at each iteration        (WRITTEN)

            void Reset(void); //      (WRITTEN)


        private:
            // Methods
            void QueueThread(void);
            void commandsCallback(const robot4ws_msgs::Dynamixel_parameters1::ConstPtr &dyn_msg); //      (WRITTEN)
            void loadParameters(void); //      (WRITTEN)
            void initializeMotorsMapping(void);//      (WRITTEN)
            void applyCommands(void);//         (WRITTEN)
            bool initializeControllers(void);//     (WRITTEN)


            void printParams(void);







            bool print_debug = false;
            bool use_odometry = false; //This set to false. Odometry is still a TO-DO!
            bool is_plugin_running;


            bool debug_params = true;
            bool debug_speeds = true;
            bool print_commands_status = false;

            physics::ModelPtr _model;
            sdf::ElementPtr _sdf;

            //GazeboRosPtr _gazebo_ros;

            ros::NodeHandle* _ros_node;

            event::ConnectionPtr _updateConnection;


            //Ros related attributes
            ros::Subscriber _dinamixels_sub;
            boost::shared_ptr<tf::TransformBroadcaster> _tf_broadcaster;

            nav_msgs::Odometry _odom_msg;


            boost::mutex lock;


            // PID Initialization
            common::PID _pid_vel;
            common::PID _pid_pos;

            // Joints container
            std::vector<physics::JointPtr> _joints;



            std::string _tf_prefix;
            std::string _dinamixel_topic_name;
            std::string _odo_topic_name;
            std::string _odometry_frame_name;
            double _ctrl_rate;
            double _odom_update_rate;

            std::map<int, std::string> _motor_mapping;
            std::map<int, std::string> _joint_mapping;
            

            //Custom callback queue
            ros::CallbackQueue _queue;
            boost::thread _callback_queue_thread;

            common::Time _last_command_received_time;
            common::Time _last_data_sent;

            // Last commands received container
            double _last_filtered_commands[MOTORS] = {0,};
            



    };
    GZ_REGISTER_MODEL_PLUGIN(ArchimedeKinematicsPlugin)
}
#endif //ARCHIMEDE_KINEMATICS_PLUGIN_H
