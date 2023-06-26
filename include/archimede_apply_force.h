/**
* @author Marco Giberna
* @email: marco.giberna@studenti.units.it
* @email: gibimarco@gmail.com
**/


#include <string>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <stdio.h>
#include <fstream>
#include <iostream>

// Include ROS related stuff
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>


namespace gazebo
{
    class ArchimedeApplyForce : public ModelPlugin
    {
        public:
            ArchimedeApplyForce(); // Constructor

            virtual ~ArchimedeApplyForce(); //Destructor

        /*
        Brief load the sensor plugin
        input:
            * _sensor: Pointer to the sensor that loaded this plugin
            * _sdf: SDF element that describes the plugin
        output:
            * None
        */

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

            virtual void OnUpdate(void); // Callback that receive the contact sensor's updare signal
        

        private:
            bool debug_mode = true;

            bool print_debug_ = true;

            physics::ModelPtr model;

            physics::WorldPtr world;

            physics::LinkPtr link_;

            sdf::ElementPtr sdf;

            transport::NodePtr node_pt; //Node for gazebo communication

            event::ConnectionPtr onnection;

            //Publisher
            transport::PublisherPtr pub;

            std::string link_name_; 

            ignition::math::Vector3d commanded_force;

            

            


    };

    GZ_REGISTER_MODEL_PLUGIN(ArchimedeApplyForce)
}