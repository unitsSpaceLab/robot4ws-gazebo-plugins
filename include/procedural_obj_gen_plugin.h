/**
* @author Matteo Caruso
* @email: matteo.caruso@phd.units.it
* @email: matteo.caruso1993@gmail.com
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



namespace gazebo
{
    class proceduralObjGen : public ModelPlugin
    {
        public:
            proceduralObjGen(); // Constructor

            ~proceduralObjGen(); // Destructor

            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

            virtual void OnUpdate(void);

            bool spawnObj(double x, double y);

        private:
            bool getPolygon(double x, double y);

            physics::ModelPtr model;

            physics::ModelPtr terrainModel;

            sdf::ElementPtr sdf;

            physics::WorldPtr world;

            physics::LinkPtr linkTerrain;
    };
    GZ_REGISTER_MODEL_PLUGIN(proceduralObjGen);
}