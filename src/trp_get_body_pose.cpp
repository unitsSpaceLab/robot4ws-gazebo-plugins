#include "../include/trp_get_body_pose.h"



using namespace gazebo;

//GZ_REGISTER_MODEL_PLUGIN(TRPGetJointStatePlugin);


TRPGetBodyPosePlugin::TRPGetBodyPosePlugin() : ModelPlugin()
{
    //Store the pointer to the model
    if (this -> print_debug_)
    {
        std::cerr << "TRP Get Body Pose Plugin Constructor called...\n";
    }
}

TRPGetBodyPosePlugin::~TRPGetBodyPosePlugin()
{
    this -> data_file.close();
    if (this -> print_debug_)
    {
        std::cerr << "TRP Get Body Pose Plugin Destructor called...\n";
    }
}

void TRPGetBodyPosePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    if (this -> print_debug_)
    {
        std::cerr << "Initializing TRP Get Body Pose Plugin...\n";
    }

    
    // Get the parent sensor
    this -> model =_parent;
    this -> sdf = _sdf;
    this -> world = this -> model -> GetWorld();

    char buff[FILENAME_MAX];

    getcwd(buff, FILENAME_MAX);

    if (_sdf->HasElement("saveDir"))
    {
        this -> save_dir = _sdf->GetElement(
            "saveDir")->Get<std::string>();
    }
    else
    {
        this -> save_dir = std::string(buff);
    }

    if (_sdf->HasElement("linkName"))
    {
        this -> link_name_ = _sdf -> GetElement("linkName") -> Get<std::string>();
    }

    if (this -> print_debug_)
    {
        std::cerr << "Save Dir path is:\t" << this -> save_dir << '\n'; // (1)
        std::cerr << "Link Name is:\t" << this -> link_name_ << '\n';
        std::cerr << "Current path is:\t" << buff << '\n'; // (1)
    }

    // Get Model Namespace
    this -> link_ = this -> model->GetLink(this -> link_name_);

    if (!link_)
    {
        std::cerr << "No Link Found in the Model! Calling Plugin Destructor..." << '\n';
        //TRPGetJointStatePlugin::~TRPGetJointStatePlugin();
    }
    else
    {
        initFileWrite();
        this->updateConnection =
            event::Events::ConnectWorldUpdateBegin ( boost::bind ( &TRPGetBodyPosePlugin::OnUpdate, this ) );
    }


}

void TRPGetBodyPosePlugin::OnUpdate(void)
{
    updateFileWrite();
}

void TRPGetBodyPosePlugin::initFileWrite(void)
{
    std::string filename = this -> save_dir + "/link_data/state/" + this -> link_name_ + ".csv";
    const char * c = filename.c_str();
    if (this -> print_debug_)
    {
        std::cerr << "FILENAME IS:\t" << filename << "\n";
    }

    this -> data_file.open(c);
    this -> data_file << "time,x,y,z,roll,pitch,yaw,lin_vel_x,lin_vel_y,lin_vel_z,ang_vel_x,ang_vel_y,ang_vel_z\n";
}

void TRPGetBodyPosePlugin::updateFileWrite(bool print_values)
{
    char buffer[256];
    double time;

    //std::cerr << this -> link_ -> GetWorldPose() << '\n';
#if GAZEBO_MAJOR_VERSION >= 8
    time = this -> world -> SimTime().Double();
#else
    time = this -> world -> GetSimTime().Double();
#endif
    //math::Pose pose = this -> link_ -> GetWorldPose();

    ignition::math::Pose3d pose;
    ignition::math::Vector3d vel;
    ignition::math::Vector3d ang_vel;

#if GAZEBO_MAJOR_VERSION >= 8
    pose = this -> link_ -> WorldCoGPose();
    vel = this -> link_ -> WorldCoGLinearVel();
    ang_vel = this -> link_ -> WorldAngularVel();
#else
    pose = this -> link_ -> GetWorldCoGPose().Ign();
    vel = this -> link_ -> GetWorldCoGLinearVel().Ign();
    ang_vel = this -> link_ -> GetWorldAngularVel().Ign();
#endif

    snprintf(buffer, sizeof(buffer), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", time, pose.Pos().X(), pose.Pos().Y(), 
            pose.Pos().Z(), pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw(), vel.X(), vel.Y(), vel.Z(), ang_vel.X(), ang_vel.Y(), ang_vel.Z());
    this -> data_file << buffer;
    if (print_values)
    {
        std::cerr << buffer;
    }
    
    
}
