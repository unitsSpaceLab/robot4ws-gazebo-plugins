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


namespace gazebo
{
    class ArchimedeGetLinkPose : public ModelPlugin
    {
        public:
            ArchimedeGetLinkPose(); // Constructor

            virtual ~ArchimedeGetLinkPose(); //Destructor

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
            void initFileWrite(void);
            void updateFileWrite(bool print_values = false);

            bool debug_mode = false;

            bool print_debug_ = false;

            bool _readCOG = false;

            bool _world_mode = true;

            physics::ModelPtr model;

            physics::WorldPtr world;

            physics::LinkPtr link_;

            sdf::ElementPtr sdf;

            transport::NodePtr node_pt; //Node for gazebo communication

            event::ConnectionPtr updateConnection;

            //Publisher
            transport::PublisherPtr pub;

            std::string save_dir;

            std::string link_name_;

            std::ofstream data_file;

            

            


    };

    GZ_REGISTER_MODEL_PLUGIN(ArchimedeGetLinkPose)
}