#include "../include/archimede_kinematics_plugin.h"


using namespace gazebo;


ArchimedeKinematicsPlugin::ArchimedeKinematicsPlugin() : ModelPlugin()
{
    ROS_INFO("Starting Archimede kinematics ROS-Gazebo plugin");
    this -> _joints.resize(MOTORS);
}

ArchimedeKinematicsPlugin::~ArchimedeKinematicsPlugin()
{
    ROS_INFO("Shutting down Archimede kinematics plugin");
    this -> _queue.clear();
    this -> _queue.disable();

    //this -> _gazebo_ros -> node() -> shutdown();
    this -> _ros_node -> shutdown();
    this -> _callback_queue_thread.join();

    this -> _joints.clear();
    this -> _dinamixels_sub.shutdown();
    ROS_INFO("Archimede initial start done!");

}



void ArchimedeKinematicsPlugin::loadParameters(void)
{
    this -> _ros_node -> param<std::string>("dinamixel_topic_name", this -> _dinamixel_topic_name, "cmd_vel_motors");
    this -> _ros_node -> param<double>("ctrl_rate", this -> _ctrl_rate, 10);
    this -> _ros_node -> param<std::string>("odom_topic_name", this -> _odo_topic_name, "odom");
    this -> _ros_node -> param<std::string>("odom_frame_name", this -> _odometry_frame_name, "odom");
    this -> _ros_node -> param<double>("odom_rate", this -> _odom_update_rate, 10);
    


    /*this -> _gazebo_ros -> getParameter<std::string>(this -> _dinamixel_topic_name, "dinamixel_topic_name", "cmd_vel_motors");
    this -> _gazebo_ros -> getParameter<std::string>(this -> _odo_topic_name, "odom_topic_name", "odom");
    this -> _gazebo_ros -> getParameter<std::string>(this -> _odometry_frame_name, "odom_frame_name", "odom");
    this -> _gazebo_ros -> getParameter<double>(this -> _odom_update_rate, "odom_rate", 10);*/

    double pid_velocity_p, pid_velocity_d, pid_velocity_i;
    double pid_position_p, pid_position_d, pid_position_i;

    this -> _ros_node -> param<double>("pid_values_velocity_p", pid_velocity_p, 20);
    this -> _ros_node -> param<double>("pid_values_velocity_d", pid_velocity_d, 0.0);
    this -> _ros_node -> param<double>("pid_values_velocity_i", pid_velocity_i, 0.0);


    /*this -> _gazebo_ros -> getParameter<double>(pid_velocity_p, "pid_values_velocity_p", 0.1);
    this -> _gazebo_ros -> getParameter<double>(pid_velocity_d, "pid_values_velocity_d", 0);
    this -> _gazebo_ros -> getParameter<double>(pid_velocity_i, "pid_values_velocity_i", 0);*/


    this -> _pid_vel = common::PID(pid_velocity_p, pid_velocity_i, pid_velocity_d);

    this -> _ros_node -> param<double>("pid_values_position_p", pid_position_p, 20);
    this -> _ros_node -> param<double>("pid_values_position_d", pid_position_d, 0.0);
    this -> _ros_node -> param<double>("pid_values_position_i", pid_position_i, 0.0);

    /*this -> _gazebo_ros -> getParameter<double>(pid_position_p, "pid_values_position_p", 0.1);
    this -> _gazebo_ros -> getParameter<double>(pid_position_d, "pid_values_position_d", 0);
    this -> _gazebo_ros -> getParameter<double>(pid_position_i, "pid_values_position_i", 0);*/


    this -> _pid_pos = common::PID(pid_position_p, pid_position_i, pid_position_d);

    ROS_INFO("Parameters loaded correctly");

    if (this -> debug_params)
    {
        this -> printParams();
    }

}

void ArchimedeKinematicsPlugin::printParams(void)
{
    std::cerr << "************************************************************************************" << "\n";
    std::cerr << "Param with name [" << "dinamixel_topic_name]:\t" << this -> _dinamixel_topic_name << "\n";
    std::cerr << "Param with name [" << "odom_topic_name]:\t" << this -> _odo_topic_name << "\n";
    std::cerr << "Param with name [" << "odom_frame_name]:\t" << this -> _odometry_frame_name << "\n";
    std::cerr << "Param with name [" << "odom_rate]:\t" << this -> _odom_update_rate << "\n";
    std::cerr << "Param with name [" << "pid_values_velocity_p]:\t" << this -> _pid_vel.GetPGain() << "\n";
    std::cerr << "Param with name [" << "pid_values_velocity_d]:\t" << this -> _pid_vel.GetDGain() << "\n";
    std::cerr << "Param with name [" << "pid_values_velocity_i]:\t" << this -> _pid_vel.GetIGain() << "\n";
    std::cerr << "Param with name [" << "pid_values_position_p]:\t" << this -> _pid_pos.GetPGain() << "\n";
    std::cerr << "Param with name [" << "pid_values_position_d]:\t" << this -> _pid_pos.GetDGain() << "\n";
    std::cerr << "Param with name [" << "pid_values_position_i]:\t" << this -> _pid_pos.GetIGain() << "\n";
    std::cerr << "************************************************************************************" << "\n";
}



void ArchimedeKinematicsPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    ROS_INFO("Initializing Archimede kinematics plugin...");

    this -> _model = model;
    
    this -> _sdf = sdf;

    //this -> _gazebo_ros = GazeboRosPtr(new GazeboRos(this -> _model, this -> _sdf, "ArchimedeKinematicsPlugin"));
    this -> _ros_node = new ros::NodeHandle();


    //this -> _ros_node ->ok();
    //this -> _gazebo_ros -> isInitialized();


    //Initialize motors mapping
    this -> initializeMotorsMapping();

    // Load ROS parameters
    this ->loadParameters();


    // Initialize PID controllers
    this -> initializeControllers();



    ros::SubscribeOptions sub_opt = ros::SubscribeOptions::create<robot4ws_msgs::Dynamixel_parameters1>(this -> _dinamixel_topic_name,
                                     100, boost::bind(&ArchimedeKinematicsPlugin::commandsCallback, this, _1), ros::VoidPtr(), &this -> _queue);

    this -> _dinamixels_sub = this -> _ros_node  -> subscribe(sub_opt);
    ROS_INFO("Subscribing to topic [%s]", this -> _dinamixel_topic_name.c_str());


    // Start custom queue

    this -> _callback_queue_thread = boost::thread(boost::bind(&ArchimedeKinematicsPlugin::QueueThread, this));

    // Listen to update event
    this -> _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ArchimedeKinematicsPlugin::OnUpdate, this));

    this -> is_plugin_running = true;

    ROS_INFO("Archimede kinematics plugin loaded corrrectly");

}

void ArchimedeKinematicsPlugin::initializeMotorsMapping(void)
/**
* Description of the function/method
*
* @param <param> Description of the parameter
* @return Description of the return value
*/
{
    //Write here the motors mapping
    this -> _motor_mapping = {{0, "Archimede_br_drive_joint"},
                                {1, "Archimede_fr_drive_joint"},
                                {2, "Archimede_bl_drive_joint"},
                                {3, "Archimede_fl_drive_joint"},
                                {4, "Archimede_br_steer_joint"},
                                {5, "Archimede_fr_steer_joint"},
                                {6, "Archimede_bl_steer_joint"},
                                {7, "Archimede_fl_steer_joint"}};
    
}


bool ArchimedeKinematicsPlugin::initializeControllers(void)
{

    //Here initialize the controllers!
    if (this -> _model -> GetJointCount() == 0)
    {
        ROS_ERROR("No joints have been found... Please check the model");
        return false;
    }
    else if (this -> _model -> GetJointCount() != 0 && this -> _model -> GetJointCount() != MOTORS)
    {
        ROS_ERROR("Invalid number of joints... Check the model");
        return false;
    }


    for (int i=0; i < MOTORS; i++)
    {
        this -> _joints[i] = this -> _model -> GetJoint(this -> _motor_mapping[i]);

        //Apply here a PID controller to the joint
        if (i < MOTORS/2)
        {
            this -> _model -> GetJointController() -> SetVelocityPID(this -> _joints[i] -> GetScopedName(), this -> _pid_vel);
            this -> _model -> GetJointController() -> SetVelocityTarget(this -> _joints[i] -> GetScopedName(), this -> _last_filtered_commands[i]);
        }
        else
        {
            this -> _model -> GetJointController() -> SetPositionPID(this -> _joints[i] -> GetScopedName(), this -> _pid_pos);
            this -> _model -> GetJointController() -> SetPositionTarget(this -> _joints[i] -> GetScopedName(), this -> _last_filtered_commands[i]);
        }

    }
}


void ArchimedeKinematicsPlugin::Reset(void)
{
#if GAZEBO_MAJOR_VERSION >=8
    this -> _last_command_received_time = this -> _model -> GetWorld() -> SimTime();
#else
    this -> _last_command_received_time = this -> _model -> GetWorld() -> GetSimTime();
#endif
    for (int i=0; i < MOTORS; i++)
    {
        this -> _last_filtered_commands[i] = 0;
    }
}


void ArchimedeKinematicsPlugin::OnUpdate(void)
{
#if GAZEBO_MAJOR_VERSION >=8
    common::Time time_now = this -> _model -> GetWorld() -> SimTime();
#else
    common::Time time_now = this -> _model -> GetWorld() -> GetSimTime();
#endif


    //Apply last received commands
    //this -> applyCommands();
    double status;
    double target;
    double diff;
    if (this -> print_commands_status)
    {
        std::cout << "************************************************************************\n";
        for (int i=0; i < this -> _joints.size(); i++)
        {
            target = this -> _last_filtered_commands[i];
            if (i < MOTORS/2)
            {
                status = this -> _joints[i] -> GetVelocity( 0 );
            }
            else
            {
                status = this -> _joints[i] -> Position( 0 );
            }
            diff = target-status;
            std::cerr << this -> _joints[i] -> GetName() << "\t commanded value:  [ " <<  target << " ] \t current value: [ " << status
             << " ] \t error: [ " << diff << " ]\n";
        }
    }
    this -> _model -> GetJointController() -> Update();
    //std::cerr << "gazebo_plugin_running";


}


void ArchimedeKinematicsPlugin::commandsCallback(const robot4ws_msgs::Dynamixel_parameters1::ConstPtr &dyn_msg)
{
    //std::cerr << "pippo\n";
    boost::mutex::scoped_lock scoped_lock(this -> lock);
    
    // Fill the list with the last received commands
    this -> _last_filtered_commands[0] = -dyn_msg -> One_Primary;
    this -> _last_filtered_commands[1] = -dyn_msg -> Two_Primary;
    this -> _last_filtered_commands[2] = dyn_msg -> Three_Primary;
    this -> _last_filtered_commands[3] = dyn_msg -> Four_Primary;
    this -> _last_filtered_commands[4] = dyn_msg -> Five_Primary;
    this -> _last_filtered_commands[5] = dyn_msg -> Six_Primary;
    this -> _last_filtered_commands[6] = dyn_msg -> Seven_Primary;
    this -> _last_filtered_commands[7] = dyn_msg -> Eight_Primary;

    for (int i=0; i<8; i++)
    {
        std::cerr << this -> _last_filtered_commands[i] << "\t";
        if (i==7)
        {
            std::cerr << "\n";
        }
    }

    this -> applyCommands();
}





void ArchimedeKinematicsPlugin::applyCommands(void)
{
    for (int i=0; i < MOTORS; i++)
    {
        //Apply here a PID controller to the joint
        if (i < MOTORS/2)
        {
            //std::cerr << this -> _joints[i] -> GetScopedName() << ":\t" << this -> _last_filtered_commands[i] << "\t";
            bool success = this -> _model -> GetJointController() -> SetVelocityTarget(this -> _joints[i] -> GetScopedName(), this -> _last_filtered_commands[i]);
            /*if (success)
            {
                std::cerr<<"success\n";
            }*/
        }
        else
        {
            this -> _model -> GetJointController() -> SetPositionTarget(this -> _joints[i] -> GetScopedName(), this -> _last_filtered_commands[i]);
        }

    }
}



void ArchimedeKinematicsPlugin::QueueThread(void)
{
    static const double timeout = 0.01;
    ros::Rate rate(this -> _ctrl_rate);
    while (this -> _ros_node -> ok())
    {
        if (this -> print_debug)
        {
            std::cerr << "running\n";
        }
        this -> _queue.callAvailable(ros::WallDuration(timeout));
        rate.sleep();
    }
    
}