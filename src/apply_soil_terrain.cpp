#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

namespace gazebo
{
    struct WheelTerrainData
    {
        std::string name;
        physics::LinkPtr wheel_link;
        int terrain_id;
        std::string soil_name;

        // Soil parameters
        double k, phi, rho, c, K, n;
        double k_c, k_phi;
        double X_c;
    };

    class ApplySoilTerrain : public ModelPlugin
    {
    private:
        physics::ModelPtr model;
        event::ConnectionPtr updateConnection;

        // ROS
        ros::NodeHandle* ros_node;
        ros::Publisher terrain_pub;

        // Wheels
        std::vector<WheelTerrainData> wheels;
        std::vector<std::string> wheel_names = {
            "Archimede_br_wheel_link",
            "Archimede_fr_wheel_link", 
            "Archimede_bl_wheel_link",
            "Archimede_fl_wheel_link"
        };

    public:
        ApplySoilTerrain() : ModelPlugin() {}

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
        {
            this->model = _model;

            // Initialize ROS
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "terrain_detector_node");
            }

            this->ros_node = new ros::NodeHandle();
            this->terrain_pub = this->ros_node->advertise<std_msgs::Int32MultiArray>("wheel_terrain_ids", 10);
            
            // Initialize wheels
            this->wheels.resize(wheel_names.size());
            for (size_t i = 0; i < wheel_names.size(); ++i)
            {
                wheels[i].name = wheel_names[i];
                wheels[i].wheel_link = this->model->GetLink(wheel_names[i]);

                if (!wheels[i].wheel_link)
                {
                    ROS_ERROR_STREAM("Wheel link not found: " << wheel_names[i]);
                    return;
                }
            }

            // Connect update callback
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ApplySoilTerrain::OnUpdate, this));
                
            ROS_INFO("Terrain Detector Plugin loaded successfully");
        }

        void OnUpdate()
        {
            std_msgs::Int32MultiArray terrain_msg;
            terrain_msg.data.resize(wheels.size());

            for (size_t i = 0; i < wheels.size(); ++i)
            {
                detectAndSetTerrain(i);
                terrain_msg.data[i] = wheels[i].terrain_id;
            }

            this->terrain_pub.publish(terrain_msg);
        }

        void detectAndSetTerrain(int wheel_idx)
        {
            WheelTerrainData& wheel = wheels[wheel_idx];

            // Get terrain below wheel
            std::string terrain_name;
            double terrain_distance;
            wheel.wheel_link->GetNearestEntityBelow(terrain_distance, terrain_name);

            // Map terrain name to ID and soil type
            if (terrain_name.find("terrain_blue") != std::string::npos)
            {
                wheel.terrain_id = 2;
                wheel.soil_name = "Soil_Direct_#90_sand";
                setSoilParams_Direct90Sand(wheel);
            }
            else if (terrain_name.find("terrain_red") != std::string::npos)
            {
                wheel.terrain_id = 0;
                wheel.soil_name = "Sand_marsSim";
                setSoilParams_SandMarsSim(wheel);
            }
            else if (terrain_name.find("terrain_green") != std::string::npos)
            {
                wheel.terrain_id = 1;
                wheel.soil_name = "Clay";
                setSoilParams_Clay(wheel);
            }
            else if (terrain_name.find("terrain_yellow") != std::string::npos)
            {
                wheel.terrain_id = 3;
                wheel.soil_name = "Dry_sand";
                setSoilParams_DrySand(wheel);
            }
            else
            {
                wheel.terrain_id = 0; // Default
                wheel.soil_name = "Sand_marsSim";
                setSoilParams_SandMarsSim(wheel);
            }
        }

        void setSoilParams_Direct90Sand(WheelTerrainData& wheel)
        {
            wheel.k = 8000 * pow(10, 3);
            wheel.phi = 29 * M_PI/180;
            wheel.rho = 13.03 * pow(10, 3);
            wheel.c = 1.0 * pow(10, 3);
            wheel.K = 0.021;
            wheel.X_c = 45 * M_PI/180 - wheel.phi/2;
        }

        void setSoilParams_Clay(WheelTerrainData& wheel)
        {
            wheel.k_c = 13.19 * pow(10, 3);
            wheel.k_phi = 692.15 * pow(10, 3);
            wheel.k = wheel.k_c / 0.09 + wheel.k_phi; // assuming wheel width b = 0.09
            wheel.phi = 13 * M_PI/180;
            wheel.c = 4.14 * pow(10, 3);
            wheel.K = 6 * pow(10, -3);
            wheel.n = 0.5;
            wheel.X_c = 45 * M_PI/180 - wheel.phi/2;
        }

        void setSoilParams_DrySand(WheelTerrainData& wheel)
        {
            wheel.k_c = 0.99 * pow(10, 3);
            wheel.k_phi = 1528.43 * pow(10, 3);
            wheel.k = wheel.k_c / 0.09 + wheel.k_phi; // assuming wheel width b = 0.09
            wheel.phi = 28 * M_PI/180;
            wheel.c = 1.04 * pow(10, 3);
            wheel.K = 10 * pow(10, -3);
            wheel.n = 1.10;
            wheel.X_c = 45 * M_PI/180 - wheel.phi/2;
        }

        void setSoilParams_SandMarsSim(WheelTerrainData& wheel)
        {
            wheel.k_c = 13.6 * pow(10, 3);
            wheel.k_phi = 2259.1 * pow(10, 3);
            wheel.k = wheel.k_c / 0.09 + wheel.k_phi; // assuming wheel width b = 0.09
            wheel.c = 0.4623 * pow(10, 3);
            wheel.phi = 35 * M_PI/180;
            wheel.K = 0.015;
            wheel.n = 1.0;
            wheel.X_c = 45 * M_PI/180 - wheel.phi/2;
        }

        // Getter functions for other plugins to access soil parameters
        WheelTerrainData getWheelTerrainData(int wheel_idx) const
        {
            if (wheel_idx >= 0 && wheel_idx < wheels.size())
                return wheels[wheel_idx];
            return WheelTerrainData();
        }

        ~ApplySoilTerrain()
        {
            if (this->ros_node)
            {
                this->ros_node->shutdown();
                delete this->ros_node;
            }
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(ApplySoilTerrain)
}