#include "../include/archimede_get_body_pose.h"



using namespace gazebo;

//GZ_REGISTER_MODEL_PLUGIN(TRPGetJointStatePlugin);


ArchimedeGetLinkPose::ArchimedeGetLinkPose() : ModelPlugin()
{
    //Store the pointer to the model
    if (this -> print_debug_)
    {
        std::cerr << "Archimede Get Body Pose Plugin Constructor called...\n";
    }
}

ArchimedeGetLinkPose::~ArchimedeGetLinkPose()
{
    this -> data_file.close();
    if (this -> print_debug_)
    {
        std::cerr << "Archimede Get Body Pose Plugin Destructor called...\n";
    }
}

void ArchimedeGetLinkPose::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    if (this -> print_debug_)
    {
        std::cerr << "Initializing Archimede Get Body Pose Plugin...\n";
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

    if (_sdf -> HasElement("readCOG"))
    {
        this -> _readCOG = _sdf -> GetElement("readCOG") -> Get<bool>();
    }

    if (_sdf -> HasElement("worldMode"))
    {
        this -> _world_mode = _sdf -> GetElement("worldMode") -> Get<bool>();
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
    }
    else
    {
        this -> initFileWrite();
        this -> updateConnection =
            event::Events::ConnectWorldUpdateBegin ( boost::bind ( &ArchimedeGetLinkPose::OnUpdate, this ) );
    }


}

void ArchimedeGetLinkPose::OnUpdate(void)
{
    this -> updateFileWrite();
}

void ArchimedeGetLinkPose::initFileWrite(void)
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

void ArchimedeGetLinkPose::updateFileWrite(bool print_values)
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
    if (this -> _world_mode)
    {
        if (this -> _readCOG)
        {
            if (this ->_world_mode)
            {
                pose = this -> link_ -> WorldCoGPose();
                vel = this -> link_ -> WorldCoGLinearVel();
            }
        }
        else
        {
            pose = this -> link_ -> WorldPose();
            vel = this -> link_ -> WorldLinearVel();
        }
        ang_vel = this -> link_ -> WorldAngularVel();
    }
    else
    {
        pose = this -> link_ -> RelativePose();
        vel = this -> link_ -> RelativeLinearVel();
        ang_vel = this -> link_ -> RelativeAngularVel();
    }
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
