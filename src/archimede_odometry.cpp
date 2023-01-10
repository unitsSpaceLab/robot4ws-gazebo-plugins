/**
 * @file archimede_odometry.cpp
 * @date 20/11/2022
 * 
 * 
 * 
 * @author Matteo Caruso
 * contact: matteo.caruso@phd.units.it
 * 
 * 
 * 
*/


#include "../include/archimede_odometry.h"
#include "archimede_odometry.h"

using namespace gazebo;


ArchimedeOdometryPlugin::ArchimedeOdometryPlugin() : ModelPlugin()
{
    // Class constructor for Archimede odometry plugin
    ROS_INFO("Starting Archimede ROS-Gazebo Odometry plugin...");
    this -> _joints.resize(MOTORS);
    for (int i = 0; i < MOTORS/2; i++)
    {
        this -> _last_delta_s_forward[i] = 0;
    }
    this -> _integrator = "Euler";
    this -> _step = 0;
    this -> _skip_frames = 1;

}

ArchimedeOdometryPlugin::~ArchimedeOdometryPlugin()
{
    // Class destructor
    ROS_INFO("Shutting down odometry plugin...");
    this -> _odom_pub.shutdown();
    this -> _nh->shutdown();
    this -> _joints.clear();
}

void ArchimedeOdometryPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    //Loading odometry plugin
    ROS_INFO("Loading Archimede rover odometry plugin...");
    this -> _model = model;
    this -> _sdf = sdf;
    this -> _world = this -> _model -> GetWorld();
    this -> _parseSDFparams();
    this -> initializeOdometry();

    this -> Reset();

    

    if (this -> print_joints_info)
    {
        this -> printJointsInfo();
    }

    

    //Initialize ROS related stuff
    this -> initializeROSelements();

    this -> connection = event::Events::ConnectWorldUpdateBegin(std::bind(&ArchimedeOdometryPlugin::OnUpdate, this));

    if (this -> _odom_mode == "velocity")
    {
        std::cerr << "velocity mode selected\n";
    }
    else if (this -> _odom_mode == "position")
    {
        std::cerr << "position mode selected\n";
    }


    ROS_INFO("Archimede rover odometry plugin successfully loaded!");

}

bool gazebo::ArchimedeOdometryPlugin::OnUpdate(void)
{
    static common::Time time_current_reading;
#if GAZEBO_MAJOR_VERSION >=8
    time_current_reading = this -> _world -> SimTime();
#else
    time_current_reading = this -> _world -> GetSimTime();
#endif
    this -> _step += 1;

    if (!(this -> _step % this -> _skip_frames == 0))
    {
        return false;
    }

    

    // Read Wheels Encoders
    std::pair<std::vector<double>, std::vector<double>> res = this -> readEncoders();

    //first=wheels && second=steering


    double delta_t = (time_current_reading - this ->last_update_time).Double();
    double current_ds[MOTORS/2], diff_pos[MOTORS/2];



    if (this -> print_wheel_readings)
    {
        std::cout << "Delta T= " << (time_current_reading - this ->last_update_time).Double() << "\n";
        for (int i=0; i < res.first.size(); i++)
        {
            std::cout << "Steer= " << res.second[i] << "\t Wheels= " << res.first[i] << "\n";
        }
        std::cout << "###################################################\n";
    }


    //Update the odometry here....




    //Update B matrix
    if (this -> _integrator == "Euler")
        for (int i=0; i < res.first.size(); i++)
        {
            this -> B_Matrix[i][0] = this -> _joint_mapping[i].geom.R * res.first[i] * cos(res.second[i]);
            this -> B_Matrix[i+4][0] = this -> _joint_mapping[i].geom.R * res.first[i]*sin(res.second[i]);
        }
    else if (this -> _integrator == "Trapezi")
    {
        if (this -> _odom_mode == "velocity")
        {
            for (int i=0; i < res.first.size(); i++)
            {
                this -> B_Matrix[i][0] = 0.5*(this -> _joint_mapping[i].geom.R * res.first[i] * cos(res.second[i]) +
                this -> _joint_mapping[i].geom.R * this ->_last_readings.first[i] * cos(this -> _last_readings.second[i]));
                this -> B_Matrix[i+4][0] = 0.5*(this -> _joint_mapping[i].geom.R * res.first[i] * sin(res.second[i]) +
                this -> _joint_mapping[i].geom.R * this ->_last_readings.first[i] * sin(this -> _last_readings.second[i]));
            }
        }
        else if (this -> _odom_mode == "position")
        {
            for (int i=0; i < res.first.size(); i++)
            {
                diff_pos[i] = res.first[i] - this -> _last_readings.first[i];
                current_ds[i] = this -> _joint_mapping[i].geom.R * diff_pos[i];
            }

            for (int i=0; i < res.first.size(); i++)
            {
                this -> B_Matrix[i][0] = 0.5 * (this -> _last_delta_s_forward[i] * cos( this -> _last_readings.second[i]) +
                    current_ds[i] * cos(res.second[i]));
                
                this -> B_Matrix[i+4][0] = 0.5 * (this -> _last_delta_s_forward[i] * sin( this -> _last_readings.second[i] ) +
                    current_ds[i] * sin(res.second[i]));
            }
        }
    }
    else
    {
        std::cout << "Not supported!\n";
        this -> ~ArchimedeOdometryPlugin();
    }

    if (this -> _odom_mode == "velocity")
    {
        std::vector<std::vector<double> > x_dot = MatrixMultiplication(this -> P_inv_A, this -> B_Matrix);
        
        //showMatrix(x_dot);

        ignition::math::Vector3d vel = ignition::math::Vector3d(x_dot[0][0], x_dot[1][0], x_dot[2][0]);
        

        ignition::math::Vector3<double> lin_vel(x_dot[0][0], x_dot[1][0], 0);

        

        double ang_vel = x_dot[2][0];

        double new_theta = ang_vel*delta_t + this -> last_theta;

        //double tmp_theta = (this -> last_theta + new_theta)/2;
        double tmp_theta = this -> last_theta;


        
        static ignition::math::Quaternion<double> q;
        q.Euler(ignition::math::Vector3d(0,0,tmp_theta));

        if (this -> print_debug_update)
        {
            std::cout << "Computed x_dot vector\t";
            std::cout << "x_dot: " << x_dot[0][0] << "\t y_dot: " << x_dot[1][0] << "\t theta_dot: " << x_dot[2][0] << "\n";
            std::cout << "Ignition x_dot vector\t";
            std::cout << "x_dot: " << vel.X() << "\t y_dot: " << vel.Y() << "\t theta_dot: " << vel.Z() << "\n";
            std::cout << "Ignition linear velocity in local frame\t";
            std::cout << "x_dot: " << lin_vel.X() << "\t y_dot: " << lin_vel.Y() << "\t z_dot: " << lin_vel.Z() << "\n";
            std::cout << "last_theta: " << this -> last_theta << "\ttmp_theta: " << tmp_theta << "\tnew_theta: " << new_theta <<"\n";
            std::cout << "Ignition quaternion:\t";
            std::cout << "q_x: "<< q.X() << "\tq_y: "<< q.Y() << "\tq_z: " << q.Z() << "\tq_w: " << q.W() <<"\n";
        }

        


        //Rotate local speed into inertial speed
        ignition::math::Vector3d inertial_vel = q.RotateVector(lin_vel);

        if (this -> print_debug)
        {
            std::cout << "x_dot: " << inertial_vel.X() << "\t y_dot: " << inertial_vel.Y() << "\t z_dot: " << inertial_vel.Z() << "\n";
        }

        this -> last_pose += inertial_vel*delta_t;
        this -> last_theta = new_theta;
    }
    else if (this -> _odom_mode == "position")
    {
        std::vector<std::vector<double> > x = MatrixMultiplication(this -> P_inv_A, this -> B_Matrix); //Delta x in local coordinates
        
        //showMatrix(x_dot);

        ignition::math::Vector3d x_ign = ignition::math::Vector3d(x[0][0], x[1][0], x[2][0]);
        

        ignition::math::Vector3<double> x_ign_real(x[0][0], x[1][0], 0);

        

        double delta_theta = x[2][0];


        //double tmp_theta = (this -> last_theta + new_theta)/2;
        double tmp_theta = this -> last_theta;


        
        static ignition::math::Quaternion<double> q;
        q.Euler(ignition::math::Vector3d(0,0,tmp_theta));

        


        //Rotate local speed into inertial speed
        ignition::math::Vector3d inertial_pos = q.RotateVector(x_ign_real);

        for (int i=0; i < MOTORS/2; i++)
        {
            this -> _last_delta_s_forward[i] = current_ds[i];
        }


        this -> last_pose += inertial_pos;
        this -> last_theta += delta_theta;
    }

    this -> last_update_time = time_current_reading;
    this -> _last_readings = res;

    //std::cout << this -> last_pose << "\n";

    this -> publishTF();

    if (this -> publish_body_real_frame)
    {
        this -> publishRealTF();
    }


    if ((time_current_reading - this -> last_time_published).Double() >= 1/this -> _odom_config.odom_publish_rate)
    {
        this -> publishOdom();
        this -> last_time_published = time_current_reading;
    }

    return true;


}

void gazebo::ArchimedeOdometryPlugin::Reset(void)
{
    //Reset some values
    ROS_INFO("Resetting odometry plugin...");

    this -> last_theta = 0;
    this -> _odom_msg = nav_msgs::Odometry();
    for (int i = 0; i < MOTORS/2; i++)
    {
        this -> _last_delta_s_forward[i] = 0;
    }

    this -> _step = 0;

    this -> _last_readings = this -> readEncoders();
    this -> last_pose = ignition::math::Vector3d();

#if GAZEBO_MAJOR_VERSION>=8
    this -> last_time_published = this -> _world -> SimTime();
    this -> last_update_time = this -> _world -> SimTime();
#else
    this -> last_time_published = this -> _world -> GetSimTime();
    this -> last_update_time = this -> _world -> GetSimTime();
#endif
    
}

void gazebo::ArchimedeOdometryPlugin::publishOdom(void)
{
    static ignition::math::Quaternion<double> q;
    q.Euler(ignition::math::Vector3d(0,0,this -> last_theta));
    
    this -> _odom_msg.child_frame_id = this -> _odom_config.base_link_frame_name;
    this -> _odom_msg.header.stamp = ros::Time::now();
    this -> _odom_msg.header.frame_id = this -> _odom_config.odom_frame_name;
    this -> _odom_msg.pose.pose.position.x = this -> last_pose.X();
    this -> _odom_msg.pose.pose.position.y = this -> last_pose.Y();
    this -> _odom_msg.pose.pose.position.z = this -> last_pose.Z();


    this -> _odom_msg.pose.pose.orientation.w = q.W();
    this -> _odom_msg.pose.pose.orientation.x = q.X();
    this -> _odom_msg.pose.pose.orientation.y = q.Y();
    this -> _odom_msg.pose.pose.orientation.z = q.Z();

    this -> _odom_pub.publish(this -> _odom_msg);
}

void ArchimedeOdometryPlugin::initializeROSelements(void)
{
    ROS_INFO("Starting Archimede Odometry ROS node...");

    //Start the ROS node
    this -> _nh = new ros::NodeHandle();
    //ros::AdvertiseOptions pub_options = ros::AdvertiseOptions::create("odom",100);


    this -> _nh -> param<bool>("publish_real_tf", this -> publish_body_real_frame, false);


    // Initialize the odometry publisher
    this -> _odom_pub = this -> _nh -> advertise<nav_msgs::Odometry>(this -> _odom_config.odom_topic_name,this -> _odom_config.odom_que_size);

    // Initialize the TF transform broadcaster
    this -> _tf = tf::TransformBroadcaster();

    if (this -> _odom_config.fake_publishers.enable)
    {
        this -> _fake_pub_list.push_back(this -> _nh -> advertise<robot4ws_msgs::Motor_current_int>(this -> _odom_config.fake_publishers.motorCurrentTopicName, 1));
        this -> _fake_pub_list.push_back(this -> _nh -> advertise<robot4ws_msgs::Motor_temperature_int>(this -> _odom_config.fake_publishers.motorTemperatureTopicName,1));
        this -> _fake_pub_list.push_back(this -> _nh -> advertise<robot4ws_msgs::Sound>(this -> _odom_config.fake_publishers.soundTopicName,1));
        this -> _fake_pub_list.push_back(this -> _nh -> advertise<robot4ws_msgs::SensorState>(this -> _odom_config.fake_publishers.sensorStateTopicName, 1));
    }
}

std::vector<double> ArchimedeOdometryPlugin::getWheelsState(void)
{
    static std::vector<double> data(4);

    for (int i=0; i< this ->_joint_mapping.size(); i++)
    {
        if (this -> _odom_mode == "velocity")
        {
            data[i] = this -> _model -> GetJoint( this -> _joint_mapping[i].drive_joint) -> GetVelocity( 0 ) * 
                this -> _joint_mapping[i].geom.driving_mode;
        }
        else if (this -> _odom_mode == "position")
        {
            data[i] = this -> _model -> GetJoint( this -> _joint_mapping[i].drive_joint) -> Position( 0 ) * 
                this -> _joint_mapping[i].geom.driving_mode;
        }
    }

    return data;
}

std::vector<double> gazebo::ArchimedeOdometryPlugin::getSteersState(void)
{
    static std::vector<double> data(4);

    for (int i=0; i< this ->_joint_mapping.size(); i++)
    {
        data[i] = this -> _model -> GetJoint( this -> _joint_mapping[i].steer_joint) -> Position( 0 );
    }

    return data;

}

void gazebo::ArchimedeOdometryPlugin::initializeOdometry(void)
{
    std::vector<std::string> names = {"Archimede_br_steer_joint", "Archimede_fr_steer_joint", 
    "Archimede_bl_steer_joint", "Archimede_fl_steer_joint"};
    int driving_modes[4] = {-1, -1, 1, 1};


    for (int i=0; i < MOTORS/2; i++)
    {
        joint_pair tmp;
        tmp.id = i;
        tmp.steer_joint = names[i];
        tmp.drive_joint = this -> _model -> GetJoint(names[i]) -> GetChild() -> GetChildJoints()[0] -> GetName();
        tmp.geom.R = wheel_radius;

        ignition::math::Vector3<double> pos_tmp = this -> _model -> GetJoint(names[i]) -> GetChild() -> RelativePose().Pos();

        tmp.geom.L = sqrt(pow(pos_tmp.X(),2) + pow(pos_tmp.Y(),2));
        tmp.geom.alpha = atan2(pos_tmp.Y(), pos_tmp.X());
        tmp.geom.driving_mode = driving_modes[i];



        this -> _joint_mapping.insert({i,tmp});
        
    }



    if (this -> print_mapping_debug)
    {
        std::cout << "************************************************************************************************************\n";
        for (int i=0; i < this ->_joint_mapping.size(); i++)
        {
            std::cout << "ID: [" << this ->_joint_mapping[i].id << "] \t steer_joint_name: [" << this -> _joint_mapping[i].steer_joint << 
            "] \t drive_joint_name: [" << this -> _joint_mapping[i].drive_joint <<  "] \t R: [" << this -> _joint_mapping[i].geom.R <<
            "] \t L: [" << this -> _joint_mapping[i].geom.L << "] \t alpha: [" << this -> _joint_mapping[i].geom.alpha<< "\n";
        }
        std::cout << "************************************************************************************************************\n";
    }

    this -> B_Matrix.resize(8,std::vector<double>(1)); //Initialize matrices with zero elements

    this -> A_Matrix.resize(8,std::vector<double>(3)); //Initialize matrices with zero elements

    for (int i=0; i < this -> _joint_mapping.size(); i++)
    {
        //Fill the first four rows
        this -> A_Matrix[i] = {1, 0, -this ->_joint_mapping[i].geom.L*sin( 
            this -> _joint_mapping[i].geom.alpha)};

        
        //Fill the last four rows
        this -> A_Matrix[i+4] = {0, 1, this -> _joint_mapping[i].geom.L*cos(
            this -> _joint_mapping[i].geom.alpha)};

    }

    //Compute the transpose of A
    std::vector<std::vector<double> > A_t = MatrixTranspose(this -> A_Matrix);

    //Compute the multiplication between A^T and A
    std::vector<std::vector<double> > A_Mul = MatrixMultiplication(A_t,this -> A_Matrix);

    //Compute inv(A^T * A)
    std::vector<std::vector<double> > Inv_A;
    Inverse3x3(A_Mul, Inv_A);

    //Compute pseudo-inverse
    this -> P_inv_A = MatrixMultiplication(Inv_A, A_t);

    


    if (this -> debug_matrices)
    {   std::cout << "**********************************************\n";
        std::cout << "\t A Matrix\n";
        std::cout << "**********************************************\n";
        showMatrix(this -> A_Matrix);

        std::cout << "**********************************************\n";
        std::cout << "\t B Matrix\n";
        std::cout << "**********************************************\n";
        showMatrix(this -> B_Matrix);


        std::cout << "**********************************************\n";
        std::cout << "\t A Transpose Matrix\n";
        std::cout << "**********************************************\n";
        showMatrix(A_t);

        std::cout << "**********************************************\n";
        std::cout << "\t (A^T * A)\n";
        std::cout << "**********************************************\n";
        showMatrix(A_Mul);

        std::cout << "**********************************************\n";
        std::cout << "\t inv(A^T * A)\n";
        std::cout << "**********************************************\n";
        showMatrix(Inv_A);

        std::cout << "**********************************************\n";
        std::cout << "\t pinv(A)\n";
        std::cout << "**********************************************\n";
        showMatrix(this -> P_inv_A);
    }







}

std::pair<std::vector<double>, std::vector<double>> gazebo::ArchimedeOdometryPlugin::readEncoders(void)
{
    static std::pair<std::vector<double>, std::vector<double>> out;
    std::vector<double> wheels = this -> getWheelsState();
    std::vector<double> steers = this -> getSteersState();

    
    out.first = wheels;
    out.second = steers;
    return out;

}

void gazebo::ArchimedeOdometryPlugin::publishTF(void)
{
    static geometry_msgs::TransformStamped tf_msg = geometry_msgs::TransformStamped();
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = this -> _odom_config.odom_frame_name;
    tf_msg.child_frame_id = this -> _odom_config.base_link_frame_name;

    geometry_msgs::Quaternion q_tmp = tf::createQuaternionMsgFromYaw(this -> last_theta);

    tf_msg.transform.rotation = q_tmp;

    tf_msg.transform.translation.x = this -> last_pose.X();
    tf_msg.transform.translation.y = this -> last_pose.Y();
    tf_msg.transform.translation.z = this -> last_pose.Z();

    this -> _tf.sendTransform(tf_msg);

}

void gazebo::ArchimedeOdometryPlugin::publishRealTF(void)
{
    geometry_msgs::TransformStamped tf_msg_real = geometry_msgs::TransformStamped();
    
    tf_msg_real.header.stamp = ros::Time::now();
    tf_msg_real.header.frame_id = this -> _odom_config.odom_frame_name;;
    tf_msg_real.child_frame_id = "Archimede_base_link_real";

    //Get pose real
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = this -> _model -> GetLink("Archimede_base_link") -> WorldPose();
#else
    ignition::math::Pose3d pose = this -> _model -> GetLink("Archimde_base_link") -> GetWorldPose().Ign();
#endif

    tf_msg_real.transform.translation.x = pose.Pos().X();
    tf_msg_real.transform.translation.y = pose.Pos().Y();
    tf_msg_real.transform.translation.z = pose.Pos().Z();

    tf_msg_real.transform.rotation.w = pose.Rot().W();
    tf_msg_real.transform.rotation.x = pose.Rot().X();
    tf_msg_real.transform.rotation.y = pose.Rot().Y();
    tf_msg_real.transform.rotation.z = pose.Rot().Z();

    this -> _tf.sendTransform(tf_msg_real);    

}

void ArchimedeOdometryPlugin::printJointsInfo(void)
{
    const unsigned int n_joints = this -> _model -> GetJointCount();
    std::cerr << "I have found [" << n_joints << "] joints!\n";
    physics::Joint_V joints_tmp = this -> _model -> GetJoints();

    for (int i=0; i < n_joints; i++)
    {
        physics::JointPtr joint = joints_tmp.at(i);
        physics::LinkPtr parent = joint -> GetParent();
        std::cerr << "[Joint " << i << "]\t name=" << joint -> GetName() << "\t scoped name=" << joint -> GetScopedName() << "\t number of DOFs=" << joint -> DOF() <<"\n";
        std::cerr << "Child link named [" << joint -> GetChild() -> GetName() << "] with pose relative to the parent link named [" << parent -> GetName() << "]\t xyz="<< joint -> GetChild() -> RelativePose().Pos() << "\t rpy="<< joint -> GetChild() -> RelativePose().Rot() << "\n";
    
        //std::cerr << joint -> GetChild() -> GetChildJoints()[0] -> GetName() <<"\n";
    }
}

void gazebo::ArchimedeOdometryPlugin::_parseSDFparams(void)
{

    if (this -> _sdf -> HasElement("integrator"))
    {
        this -> _integrator = this -> _sdf -> GetElement("integrator") -> Get<std::string>();
    }
    else
    {
        this -> _integrator = "Euler";
    }




    if (this -> _sdf -> HasElement("odomTopicName"))
    {
        this -> _odom_config.odom_topic_name = this -> _sdf -> GetElement("odomTopicName") -> Get<std::string>();
    }
    else
    {
        this -> _odom_config.odom_topic_name = "odom";
    }

    if (this -> _sdf -> HasElement("odomQueSize"))
    {
        this -> _odom_config.odom_que_size = this -> _sdf -> GetElement("odomQueSize") -> Get<int>();
    }
    else
    {
        this -> _odom_config.odom_que_size = 100;
    }

    if (this -> _sdf -> HasElement("odomFrameName"))
    {
        this -> _odom_config.odom_frame_name = this -> _sdf -> GetElement("odomFrameName") -> Get<std::string>();
    }
    else
    {
        this -> _odom_config.odom_frame_name = "odom";
    }

    if (this -> _sdf -> HasElement("odomPublishRate"))
    {
        this -> _odom_config.odom_publish_rate = this -> _sdf -> GetElement("odomPublishRate") -> Get<double>();
    }
    else
    {
        this -> _odom_config.odom_publish_rate = 10;
    }



    if (this -> _sdf -> HasElement("baseLinkFrameName"))
    {
        this -> _odom_config.base_link_frame_name = this -> _sdf -> GetElement("baseLinkFrameName") -> Get<std::string>();
    }
    else
    {
        this -> _odom_config.base_link_frame_name = "Archimede_base_link";
    }

    if (this -> _sdf -> HasElement("useFakePublishers"))
    {
        this -> _odom_config.fake_publishers.enable = this -> _sdf -> GetElement("useFakePublishers") -> Get<bool>();
    }
    else
    {
        this -> _odom_config.fake_publishers.enable = false;
    }

    if (this -> _sdf -> HasElement("motorCurrentTopicName"))
    {
        this -> _odom_config.fake_publishers.motorCurrentTopicName = this -> _sdf -> GetElement("motorCurrentTopicName") -> Get<std::string>();
    }
    else
    {
        this -> _odom_config.fake_publishers.motorCurrentTopicName = "Motor_current";
    }

    if (this -> _sdf -> HasElement("motorTemperatureTopicName"))
    {
        this -> _odom_config.fake_publishers.motorTemperatureTopicName = this -> _sdf -> GetElement("motorTemperatureTopicName") -> Get<std::string>();
    }
    else
    {
        this -> _odom_config.fake_publishers.motorTemperatureTopicName = "Motor_temperature";
    }

    if (this -> _sdf -> HasElement("soundTopicName"))
    {
        this -> _odom_config.fake_publishers.soundTopicName = this -> _sdf -> GetElement("soundTopicName") -> Get<std::string>();
    }
    else
    {
        this -> _odom_config.fake_publishers.soundTopicName = "sound";
    }

    if (this -> _sdf -> HasElement("JointStateTopicName"))
    {
        this -> _odom_config.fake_publishers.jointStateTopicName = this -> _sdf -> GetElement("JointStateTopicName") -> Get<std::string>();
    }
    else
    {
        this -> _odom_config.fake_publishers.jointStateTopicName = "fake_joint_state";
    }


    if (this -> _sdf -> HasElement("SensorStateTopicName"))
    {
        this -> _odom_config.fake_publishers.sensorStateTopicName = this -> _sdf -> GetElement("SensorStateTopicName") ->Get<std::string>();
    }
    else
    {
        this -> _odom_config.fake_publishers.sensorStateTopicName = "sensor_state";
    }

    if (this -> _sdf -> HasElement("OdomMode"))
    {
        this -> _odom_mode = this -> _sdf -> GetElement("OdomMode") -> Get<std::string>();
    }
    else
    {
        this -> _odom_mode = "velocity";
    }

    if (this -> _sdf -> HasElement("skipFrames"))
    {
        this -> _skip_frames = this -> _sdf -> GetElement("skipFrames") -> Get<int>();
    }
    else
    {
        this -> _skip_frames = 1;
    }
}
