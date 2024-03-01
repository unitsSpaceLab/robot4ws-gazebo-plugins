/**
* @author Simone Cottiga
* @email: simone.cottiga@phd.units.it
* @email: smcotti@gmail.com
**/

#include "../include/archimede_apply_artificial_slip.h"


using namespace gazebo;

ArchimedeApplyArtificialSlip::ArchimedeApplyArtificialSlip() : ModelPlugin() 
{
  ROS_INFO("Starting archimede_apply_artificial_slip plugin");
}


ArchimedeApplyArtificialSlip::~ArchimedeApplyArtificialSlip()
{
  if (this -> check_results)
  { this -> pub_results.shutdown();}
  if (this -> validate_plugin)
  { this -> valid_plug_pub.shutdown();}
  
  this -> _joint_states_sub.shutdown();
  this -> _slip_velocities_sub.shutdown();
  this -> _ros_node -> shutdown();

  ROS_INFO("archimede_apply_artificial_slip plugin shutted down");
}


void ArchimedeApplyArtificialSlip::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO("Loading archimede_apply_artificial_slip plugin ...");
  // Store the pointer to the model
  this -> model = _model;
  this -> sdf = _sdf;

  this -> initializePluginParam();

  this -> initializeLinks();

  this -> initializeROSelements();

  ROS_INFO("Subscribing to topic [%s]", this -> slip_velocities_topic_name.c_str());

  if (this->apply_mode == "force")
  {
    this -> initializeForcePID();
  }
  
  this -> updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ArchimedeApplyArtificialSlip::OnUpdate, this));

  ROS_INFO("archimede_apply_artificial_slip plugin successfully loaded");
}


void ArchimedeApplyArtificialSlip::OnUpdate(void)
{
  /*ignition::math::Vector3d normal_vel;
  for (int i = 0; i < 4; i++)
  {
    normal_vel = this -> link[i] -> GetParentJointsLinks()[0] -> RelativeLinearVel();
  }*/
  
  if (this->apply_mode == "force")
  {
    this -> time_now = this -> model -> GetWorld() -> SimTime();

    common::Time dt = this->time_now - this->time_prev;

    ignition::math::Vector3d error, rel_real_vel;
    double error_X, error_Y, error_Z;

    for (int i = 0; i < 4; i++)
    {
      // consider the full velocity components
      this -> real_vel[i] = this -> link[i] -> WorldLinearVel();
      // only consider the XY components in wheel frame as actual velocity, ignore the Z component (ortogonal to the ground) to preserve the normal interaction
      //rel_real_vel = this -> link[i] -> RelativeLinearVel();
      //this -> real_vel[i] = this -> link[i] -> WorldPose().Rot().RotateVector(ignition::math::Vector3d(rel_real_vel.X(),rel_real_vel.Y(),0.0));

      error = this->real_vel[i] - this->target_vel[i];

      error_X = error.X();
      error_Y = error.Y();
      error_Z = error.Z();

      this -> pid_X_force[i].Update(error_X, dt);
      this -> pid_Y_force[i].Update(error_Y, dt);
      this -> pid_Z_force[i].Update(error_Z, dt);
    }

    this -> applyForce();

    this -> time_prev = this -> time_now;
  }
  else
  {
    this -> applyVelocity();
  }
}


template <class joint_states_message_type>
void ArchimedeApplyArtificialSlip::jointStateCallback(const joint_states_message_type &joint_msg)
{
  for (int i = 0; i < 4; i++)
  {
    this -> joint_velocities[i] = joint_msg -> velocity[this -> link_map[i]];     // wheels
    this -> joint_velocities[i+4] = joint_msg -> velocity[this -> link_map[i+4]]; // steers (not used)

    this -> theor_vel[i] = this -> link[i] -> GetParentJoints()[0] -> GetParent() -> 
        WorldPose().Rot().RotateVector(ignition::math::Vector3d(joint_velocities[i]*wheel_radius, 0, 0));
  }

  this -> updateTargetVelocity();

  if (this -> apply_mode == "force")
  {
    this -> resetForcePID();
  }
}


void ArchimedeApplyArtificialSlip::slipVelCallback(const SLIP_MESSAGE_TYPE::ConstPtr &vel_msg)
{
  if (this -> is_target_vel_pub)
  {
    for (int i = 0; i < 4; i++)
    {
      this -> target_vel[i].X(vel_msg -> vectors[i].x);
      this -> target_vel[i].Y(vel_msg -> vectors[i].y);
      this -> target_vel[i].Z(vel_msg -> vectors[i].z);
    }
  }
  else
  {
    for (int i = 0; i < 4; i++)
    {
      this -> target_slip[i].X(vel_msg -> vectors[i].x);
      this -> target_slip[i].Y(vel_msg -> vectors[i].y);
      this -> target_slip[i].Z(vel_msg -> vectors[i].z);
    }
  }

  if (! this -> is_target_vel_pub)
  {
    this -> updateTargetVelocity();
  }

  if (this -> apply_mode == "force")
  {
    this -> resetForcePID();
  }
}


void ArchimedeApplyArtificialSlip::applyForce(void)
{
  double force_X, force_Y, force_Z;
  ignition::math::Vector3d force_to_add_map, force_to_add_wh;
  ignition::math::Quaterniond wheel_orient;

  for (int i = 0; i < 4; i++)
  { 
    force_X = this -> pid_X_force[i].GetCmd();
    force_Y = this -> pid_Y_force[i].GetCmd();
    force_Z = this -> pid_Z_force[i].GetCmd();

    force_to_add_map = ignition::math::Vector3d(force_X, force_Y, force_Z);
    wheel_orient = this -> link[i] -> GetParentJointsLinks()[0] -> WorldPose().Rot();
    
    force_to_add_wh = wheel_orient.RotateVectorReverse(force_to_add_map);
    force_to_add_wh.Z() = 0; // set to zero the applied force Z-component in wheel frame
    force_to_add_map = wheel_orient.RotateVector(force_to_add_wh);

    this -> link[i] -> AddForce(force_to_add_map);
  }

  if(this->check_results)
  {
    this -> publishResults();
  }
  if (this -> print_results)
  {
    this -> printResults();
  }
  if (this->validate_plugin)
  {
    this -> publishPluginValidation();
  }
}


void ArchimedeApplyArtificialSlip::applyVelocity(void)
{
  for (int i = 0; i < 4; i++)
  {
    this -> link[i] -> SetLinearVel(this -> target_vel[i]);
  }

  if (this->check_results)
  {
    this -> publishResults();
  }
  if (this -> print_results)
  {
    this -> printResults();
  }
  if (this->validate_plugin)
  {
    this -> publishPluginValidation();
  }
}


void ArchimedeApplyArtificialSlip::initializePluginParam(void)
{
  if (this -> sdf -> HasElement("apply_mode"))
  {
    this -> apply_mode = this -> sdf -> GetElement("apply_mode") -> Get<std::string>();
  }
  if (this->apply_mode != "force" && this->apply_mode != "velocity")
  {
    this -> apply_mode = "force";
    //this -> apply_mode = "velocity";
  }
  ROS_INFO_STREAM("archimede_apply_artificial_slip plugin apply mode: [" << this->apply_mode << "]");

  this -> is_simulation = true; // true for gazebo simulation, false for physical rover. (just for joint_state_topic_name and link mapping)

  this -> is_target_vel_pub = true; // true: the _velocity_sub topic publishes the target velocities
                                    // false: it publishes the drift velocitites (target will be drift+commanded)

  this -> check_results = true;  // true: publish results at pub_results_topic_name topic, for check purpose
  this -> print_results = false; // true: print results on terminal, for check purpose

  this -> validate_plugin = true;
}


void ArchimedeApplyArtificialSlip::initializeLinks(void)
{
  this -> link_name = {"Archimede_br_wheel_link", "Archimede_fr_wheel_link", "Archimede_bl_wheel_link", "Archimede_fl_wheel_link"};

  for (int i = 0; i < 4; i++)
  {
    this -> link[i] = this -> model -> GetLink(this->link_name[i]);
    if (!this->link[i])
    {
      ROS_ERROR_STREAM("Link " << this->link_name[i] << "not found! Calling plugin Destructor...");
      this -> ~ArchimedeApplyArtificialSlip();
    }

    // set the link friction coefficients to zero
    auto link_friction = this -> link[i] -> GetCollision(this->link_name[i]+"_collision") -> GetSurface() -> FrictionPyramid();
    link_friction -> SetMuPrimary(0);
    link_friction -> SetMuSecondary(0);
    link_friction -> SetMuTorsion(0);
  }

  if (this -> is_simulation)
  {
    this -> link_map[0] = 2;  // BR wheel link / joint
    this -> link_map[1] = 6;  // FR wheel link / joint
    this -> link_map[2] = 0;  // BL wheel link / joint
    this -> link_map[3] = 4;  // FL wheel link / joint
    this -> link_map[4] = 3;  // BR steer link / joint
    this -> link_map[5] = 7;  // FR steer link / joint
    this -> link_map[6] = 1;  // BL steer link / joint
    this -> link_map[7] = 5;  // FL steer link / joint
  } 
  else
  {
    this -> link_map[0] = 0;  // BR wheel link / joint
    this -> link_map[1] = 1;  // FR wheel link / joint
    this -> link_map[2] = 2;  // BL wheel link / joint
    this -> link_map[3] = 3;  // FL wheel link / joint
    this -> link_map[4] = 4;  // BR steer link / joint
    this -> link_map[5] = 5;  // FR steer link / joint
    this -> link_map[6] = 6;  // BL steer link / joint
    this -> link_map[7] = 7;  // FL steer link / joint
  }
}


void ArchimedeApplyArtificialSlip::initializeForcePID(void)
{
  // Get PID values from sdf, easier to change for parameter tuning
  double K_p = this -> sdf -> GetElement("pid_kp") -> Get<double>();
  double K_i = this -> sdf -> GetElement("pid_ki") -> Get<double>();
  double K_d = this -> sdf -> GetElement("pid_kd") -> Get<double>();
  double i_max = this -> sdf -> GetElement("pid_i_max") -> Get<double>();

  // double K_p = 2200;
  // double K_i = 22000000;
  // double K_d = 0;
  // double i_max = 25;
  double i_min = -i_max;

  for (int i = 0; i < 4; i++)
  {
    this -> pid_X_force[i] = common::PID(K_p, K_i, K_d, i_max, i_min);
    this -> pid_Y_force[i] = common::PID(K_p, K_i, K_d, i_max, i_min);
    this -> pid_Z_force[i] = common::PID(K_p, K_i, K_d, i_max, i_min);
  }

  this -> time_prev = this -> model -> GetWorld() -> SimTime();
}


void ArchimedeApplyArtificialSlip::resetForcePID(void)
{
  for (int i = 0; i < 4; i++)
  {
    this -> pid_X_force[i].Reset();
    this -> pid_Y_force[i].Reset();
    this -> pid_Z_force[i].Reset(); 
  }
}


void ArchimedeApplyArtificialSlip::initializeROSelements(void)
{ 
  this -> _ros_node = new ros::NodeHandle();

  if (! this -> is_target_vel_pub)
  {
    ros::SubscribeOptions sub_opt;
    if (this -> is_simulation)
    { 
      this -> _ros_node -> param<std::string>("joint_states_topic_name", this -> joint_states_topic_name, "Archimede/joint_states");

      sub_opt = ros::SubscribeOptions::create<sensor_msgs::JointState>(this -> joint_states_topic_name, 100, 
                boost::bind(&ArchimedeApplyArtificialSlip::jointStateCallback<sensor_msgs::JointState::ConstPtr>, this, _1), ros::VoidPtr(), NULL);
    }
    else
    { 
      this -> _ros_node -> param<std::string>("joint_states_topic_name", this -> joint_states_topic_name, "joint_states");

      sub_opt = ros::SubscribeOptions::create<robot4ws_msgs::JointState1>(this -> joint_states_topic_name, 100, 
                boost::bind(&ArchimedeApplyArtificialSlip::jointStateCallback<robot4ws_msgs::JointState1::ConstPtr>, this, _1), ros::VoidPtr(), NULL);
    }

    this -> _joint_states_sub = this -> _ros_node -> subscribe(sub_opt);
  }

  this -> _ros_node -> param<std::string>("neural_network_output_topic", this -> slip_velocities_topic_name, "terrain_neural_network_out"); 

  ros::SubscribeOptions sub_opt2 = ros::SubscribeOptions::create<SLIP_MESSAGE_TYPE>(this -> slip_velocities_topic_name, 100, 
                                  boost::bind(&ArchimedeApplyArtificialSlip::slipVelCallback, this, _1), ros::VoidPtr(), NULL);

  this -> _slip_velocities_sub = this -> _ros_node -> subscribe(sub_opt2);

  if (this -> check_results)
  {
    this -> pub_results_topic_name = "check_apply_slip_plugin";
    this -> pub_results = this -> _ros_node -> advertise<robot4ws_msgs::Vector3Array>(this -> pub_results_topic_name,100);
  }

  if (this -> validate_plugin)
  {
    this -> valid_plug_topic_name = "plugin_validation_poses";
    this -> valid_plug_pub = this -> _ros_node -> advertise<geometry_msgs::PoseArray>(this -> valid_plug_topic_name,100);
  }
}


void ArchimedeApplyArtificialSlip::publishResults(void)
{
  ignition::math::Vector3d actual_vel;;

  std::string names[] = {"BR_target_vel","FR_target_vel","BL_target_vel","FL_target_vel","BR_actual_vel","FR_actual_vel","BL_actual_vel","FL_actual_vel"};

  this -> resultMsg.names.resize(8);
  this -> resultMsg.vectors.resize(8);

  this -> resultMsg.header.stamp = ros::Time::now();

  for (int i = 0; i < 4; i++)
  {
    actual_vel = this -> link[i] -> WorldLinearVel();

    this -> resultMsg.names[i] = names[i];
    this -> resultMsg.names[i+4] = names[i+4];

    this -> resultMsg.vectors[i].x = this -> target_vel[i].X();
    this -> resultMsg.vectors[i].y = this -> target_vel[i].Y();
    this -> resultMsg.vectors[i].z = this -> target_vel[i].Z();
    this -> resultMsg.vectors[i+4].x = actual_vel.X();
    this -> resultMsg.vectors[i+4].y = actual_vel.Y();
    this -> resultMsg.vectors[i+4].z = actual_vel.Z();
  }

  this -> pub_results.publish(this -> resultMsg);
}


void ArchimedeApplyArtificialSlip::printResults(void)
{
  int indx = 3;

  common::Time dt = this->time_now - this->time_prev;

  double xpe, xie, xde;
  this->pid_X_force[indx].GetErrors(xpe, xie, xde);
  double ype, yie, yde;
  this->pid_Y_force[indx].GetErrors(ype, yie, yde);
  double zpe, zie, zde;
  this->pid_Z_force[indx].GetErrors(zpe, zie, zde);

  ROS_INFO_STREAM("\n-----\t" << this->link_name[indx] << "\t-----\n"
    << "actual vel:\t" << this->real_vel[indx] << "\n"
    << "target vel:\t" << this->target_vel[indx] << "\n"
    << "time:\t" << this->time_now.Double() << "\tdt:\t" << dt.Double() << "\n"
    << "errors X :\t" << xpe << " " << xie << " " << xde << "\n"
    << "errors Y :\t" << ype << " " << yie << " " << yde << "\n"
    << "errors Z :\t" << zpe << " " << zie << " " << zde << "\n"
    << "PID out force:\t" << this->pid_X_force[indx].GetCmd() << " " << this->pid_Y_force[indx].GetCmd() << " " << this->pid_Z_force[indx].GetCmd() 
    << "\n-------------------------------------\n");
}


void ArchimedeApplyArtificialSlip::updateTargetVelocity(void)
{
  for (int i = 0; i < 4; i++)
  {
    this -> target_vel[i] = this -> theor_vel[i] + this -> target_slip[i];
  }
}


void ArchimedeApplyArtificialSlip::publishPluginValidation(void)
{
  this -> valid_plug_msg.header.stamp = ros::Time::now();
  this -> valid_plug_msg.poses.resize(4);

  ignition::math::Pose3d current_pose;
  for (int i = 0; i < 4; i++)
  {
    current_pose = this -> link[i] -> GetParentJointsLinks()[0] -> WorldPose();

    this -> valid_plug_msg.poses[i].position.x = current_pose.Pos().X();
    this -> valid_plug_msg.poses[i].position.y = current_pose.Pos().Y();
    this -> valid_plug_msg.poses[i].position.z = current_pose.Pos().Z();
    this -> valid_plug_msg.poses[i].orientation.w = current_pose.Rot().W();
    this -> valid_plug_msg.poses[i].orientation.x = current_pose.Rot().X();
    this -> valid_plug_msg.poses[i].orientation.y = current_pose.Rot().Y();
    this -> valid_plug_msg.poses[i].orientation.z = current_pose.Rot().Z();
  }

  this -> valid_plug_pub.publish(this -> valid_plug_msg);
}
