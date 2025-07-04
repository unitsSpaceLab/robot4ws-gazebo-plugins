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

  // change world update rate (-> real time factor) so that the ML model is able to properly process data
  std::string NN_model_name;
  gazebo_msgs::GetPhysicsProperties gazebo_ph_prop;
  bool has_NN_model_param = ros::param::get("/neural_network_model", NN_model_name);
  if (has_NN_model_param && NN_model_name != "none" && NN_model_name != "use_simple_slip_function" && ros::service::call<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties",gazebo_ph_prop))
  {
    gazebo_msgs::SetPhysicsProperties new_gazebo_ph_prop;
    new_gazebo_ph_prop.request.gravity = gazebo_ph_prop.response.gravity;
    new_gazebo_ph_prop.request.ode_config = gazebo_ph_prop.response.ode_config;
    new_gazebo_ph_prop.request.time_step = gazebo_ph_prop.response.time_step;
    new_gazebo_ph_prop.request.max_update_rate = 100.0;
    if (ros::service::call<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties",new_gazebo_ph_prop))
    {
      ros::param::set("/gazebo/max_update_rate",100.0);
      ROS_INFO_STREAM("gazebo max_update_rate set to " << new_gazebo_ph_prop.request.max_update_rate);
    }
  }

  this -> initializePluginParam();

  this -> initializeLinks();

  this -> initializeROSelements();

  ROS_INFO("Subscribing to topic [%s]", this -> slip_velocities_topic_name.c_str());

  if (this->apply_mode == "force")
  {
    this -> initializeForcePID();
  }

  // START this part might be needed in order to use the gazebo contacts, in case...
  // there is nothing else listening to them (ex. contact sensors or client->View->Contacts activated)
  this -> dummy_contact_node = transport::NodePtr(new transport::Node());
  this -> dummy_contact_node -> Init();
  this -> dummy_contact_sub = this -> dummy_contact_node -> Subscribe("~/physics/contacts", &ArchimedeApplyArtificialSlip::dummy_contact_callback, this);
  // END this part might be needed in order to use the gazebo contacts, in case...

  this -> updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ArchimedeApplyArtificialSlip::OnUpdate, this));

  ROS_INFO("archimede_apply_artificial_slip plugin successfully loaded");
}


void ArchimedeApplyArtificialSlip::OnUpdate(void)
{
  if (this->apply_mode == "force")
  {
    this -> time_now = this -> model -> GetWorld() -> SimTime();

    common::Time dt = this->time_now - this->time_prev;

    ignition::math::Vector3d error, rel_real_vel;

    for (int i = 0; i < 4; i++)
    {
      // consider the full velocity components
      this -> real_vel[i] = this -> link[i] -> WorldLinearVel();
      
      error = this -> real_vel[i] - this -> target_vel[i];

      this -> pid_X_force[i].Update(error.X(), dt);
      this -> pid_Y_force[i].Update(error.Y(), dt);
      this -> pid_Z_force[i].Update(error.Z(), dt);
    }

    this -> applyForce();

    this -> time_prev = this -> time_now;
  }
  else
  {
    this -> applyVelocity();
  }
}


void ArchimedeApplyArtificialSlip::slipVelCallback(const SLIP_MESSAGE_TYPE::ConstPtr &vel_msg)
{
  if (this -> saved_input_target_vels.size() >= this -> last_input_vels_to_save)
  {
    this -> saved_input_target_vels.erase(this -> saved_input_target_vels.begin());
  }
  std::array<ignition::math::Vector3d,4> temp_arr;

  for (int i = 0; i < 4; i++)
  {
    temp_arr[i].Set(vel_msg -> vectors[i].x, vel_msg -> vectors[i].y, vel_msg -> vectors[i].z);
    std::vector<ignition::math::Vector3d> filter_input({temp_arr[i]});
    for (auto& j : this -> saved_input_target_vels)
    {
      filter_input.push_back(j[i]);
    }
    this -> target_vel[i] = this -> filter_target_velocity(filter_input);
  }
  this -> saved_input_target_vels.push_back(temp_arr);

  // if (this -> apply_mode == "force")
  // {
  //   this -> resetForcePID();
  // }
}


void ArchimedeApplyArtificialSlip::applyForce(void)
{
  double force_X, force_Y, force_Z;
  ignition::math::Vector3d force_to_add_map, force_to_add_wh;
  ignition::math::Quaterniond wheel_orient;

  physics::ContactManager *contact_manager = this->model->GetWorld()->Physics()->GetContactManager();
  // ROS_INFO_STREAM("Contact manager check: " << contact_manager->GetContactCount() << "\n");
  bool wheels_in_contact[4] = {false};
  for (int i = 0; i < contact_manager->GetContactCount(); i++)
  {
    physics::Contact *contact_i = contact_manager -> GetContact(i);
    // ROS_INFO_STREAM("Contact " << i << ": " << contact_i->collision1->GetName() << "  -  " << contact_i->collision2->GetName() << "\n");

    if ((contact_i->collision1->GetModel()->GetName()==this->model->GetName()) == (contact_i->collision2->GetModel()->GetName()==this->model->GetName()))
    {
      continue;
    }

    if ((!wheels_in_contact[0])
         && (contact_i->collision1->GetName()==this->link_collision_name[0] || contact_i->collision2->GetName()==this->link_collision_name[0]))
    {
      wheels_in_contact[0] = true;
    }
    else if ((!wheels_in_contact[1])
         && (contact_i->collision1->GetName()==this->link_collision_name[1] || contact_i->collision2->GetName()==this->link_collision_name[1]))
    {
      wheels_in_contact[1] = true;
    }
    else if ((!wheels_in_contact[2])
         && (contact_i->collision1->GetName()==this->link_collision_name[2] || contact_i->collision2->GetName()==this->link_collision_name[2]))
    {
      wheels_in_contact[2] = true;
    }
    else if ((!wheels_in_contact[3])
         && (contact_i->collision1->GetName()==this->link_collision_name[3] || contact_i->collision2->GetName()==this->link_collision_name[3]))
    {
      wheels_in_contact[3] = true;
    }
  }

  for (int i = 0; i < 4; i++)
  {
    // if the wheel is in contact with no other body, don't apply any force
    if (!wheels_in_contact[i])
    {
      force_to_add_map = ignition::math::Vector3d(0, 0, 0);
    }
    else
    {
      force_X = this -> pid_X_force[i].GetCmd();
      force_Y = this -> pid_Y_force[i].GetCmd();
      force_Z = this -> pid_Z_force[i].GetCmd();

      force_to_add_map = ignition::math::Vector3d(force_X, force_Y, force_Z);
      wheel_orient = this -> link[i] -> GetParentJointsLinks()[0] -> WorldPose().Rot();

      force_to_add_wh = wheel_orient.RotateVectorReverse(force_to_add_map);
      force_to_add_wh.Z() = 0; // set to zero the applied force Z-component in wheel frame
      force_to_add_map = wheel_orient.RotateVector(force_to_add_wh);
    }

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

  this -> check_results = true;  // true: publish results at pub_results_topic_name topic, for check purpose
  this -> print_results = false; // true: print results on terminal, for check purpose

  this -> validate_plugin = true;

  this -> PIDTuning = false; // if true the PID params are defined as rosparams and can be changed while running,
                            // used to tune the PID

  // filter initialization
  if (this -> sdf -> HasElement("last_input_vels_to_save"))
  {
    this -> last_input_vels_to_save = this -> sdf -> GetElement("last_input_vels_to_save") -> Get<int>();
  }
  if (this -> last_input_vels_to_save < 1)
  {
    this -> last_input_vels_to_save = 1;
  }
  ROS_INFO_STREAM("archimede_apply_artificial_slip plugin's filter window: [" << this->last_input_vels_to_save << "]");
}


void ArchimedeApplyArtificialSlip::initializeLinks(void)
{
  this -> link_name = {"Archimede_br_wheel_link", "Archimede_fr_wheel_link", "Archimede_bl_wheel_link", "Archimede_fl_wheel_link"};
  this -> link_collision_name = {"Archimede_br_wheel_link_collision", "Archimede_fr_wheel_link_collision", "Archimede_bl_wheel_link_collision", "Archimede_fl_wheel_link_collision"};

  for (int i = 0; i < 4; i++)
  {
    this -> link[i] = this -> model -> GetLink(this->link_name[i]);
    if (!this->link[i])
    {
      ROS_ERROR_STREAM("Link " << this->link_name[i] << "not found! Calling plugin Destructor...");
      this -> ~ArchimedeApplyArtificialSlip();
    }

    // set the link friction coefficients to zero
    auto link_friction = this -> link[i] -> GetCollision(this->link_collision_name[i]) -> GetSurface() -> FrictionPyramid();
    link_friction -> SetMuPrimary(0);
    link_friction -> SetMuSecondary(0);
    link_friction -> SetMuTorsion(0);
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

  if (this -> PIDTuning)
  {
    ros::param::set("terrain_slip_plugin_PID_kp",K_p);
    ros::param::set("terrain_slip_plugin_PID_ki",K_i);
    ros::param::set("terrain_slip_plugin_PID_kd",K_d);
    ros::param::set("terrain_slip_plugin_PID_imax",i_max);
  }
}


void ArchimedeApplyArtificialSlip::resetForcePID(void)
{
  for (int i = 0; i < 4; i++)
  {
    this -> pid_X_force[i].Reset();
    this -> pid_Y_force[i].Reset();
    this -> pid_Z_force[i].Reset(); 
  }

  if (this -> PIDTuning)
  {
    double new_kp, new_ki, new_kd, new_imax;
    ros::param::get("terrain_slip_plugin_PID_kp", new_kp);
    ros::param::get("terrain_slip_plugin_PID_ki", new_ki);
    ros::param::get("terrain_slip_plugin_PID_kd", new_kd);
    ros::param::get("terrain_slip_plugin_PID_imax", new_imax);
    for (int i = 0; i < 4; i++)
    {
      pid_X_force[i].SetPGain(new_kp);
      pid_X_force[i].SetIGain(new_ki);
      pid_X_force[i].SetDGain(new_kd);
      pid_X_force[i].SetIMax(new_imax);
      pid_X_force[i].SetIMin(-new_imax);
      pid_Y_force[i].SetPGain(new_kp);
      pid_Y_force[i].SetIGain(new_ki);
      pid_Y_force[i].SetDGain(new_kd);
      pid_Y_force[i].SetIMax(new_imax);
      pid_Y_force[i].SetIMin(-new_imax);
      pid_Z_force[i].SetPGain(new_kp);
      pid_Z_force[i].SetIGain(new_ki);
      pid_Z_force[i].SetDGain(new_kd);
      pid_Z_force[i].SetIMax(new_imax);
      pid_Z_force[i].SetIMin(-new_imax);
    }
  }
}


void ArchimedeApplyArtificialSlip::initializeROSelements(void)
{ 
  this -> _ros_node = new ros::NodeHandle();

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
  ignition::math::Vector3d actual_vel;

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


ignition::math::Vector3d ArchimedeApplyArtificialSlip::filter_target_velocity(const std::vector<ignition::math::Vector3d> &input_values)
{
  int vectors_to_discard = 3;

  ignition::math::Vector3d mean_point, out;
  const int inp_size = input_values.size();
  double x=0, y=0, z=0;
  int count=0;
  std::vector<double> distance, sorted_distance;

  for (int i = 0; i < inp_size; i++)
  {
    x += input_values[i].X();
    y += input_values[i].Y();
    z += input_values[i].Z();
  }

  mean_point.Set(x/inp_size, y/inp_size, z/inp_size);

  if (vectors_to_discard >= inp_size)
  {
    return mean_point;
  }

  for (int i = 0; i < inp_size; i++)
  {
    distance.push_back(input_values[i].Distance(mean_point));
  }

  sorted_distance = distance; // sorted in descending order
  std::sort(sorted_distance.begin(), sorted_distance.end(), std::greater<double>());

  x=0; y=0; z=0;
  for (int i = 0; i < inp_size; i++)
  {
    // skip the vectors_to_discard farest points
    if (distance[i] > sorted_distance[vectors_to_discard])
    {
      continue;
    }
    x += input_values[i].X();
    y += input_values[i].Y();
    z += input_values[i].Z();
    count += 1;
  }

  out.Set(x/count, y/count, z/count);


  // std::ostringstream out_msg;
  // for (int i = 0; i < inp_size; i++)
  // {
  //   out_msg << "[" << input_values[i].X() << "; " << input_values[i].Y() << "; " << input_values[i].Z() << "] ";
  // }
  // ROS_INFO_STREAM(
  //   "\n--- last 5 vels: " << out_msg.str()
  //   << "---\n Outliers: " << (inp_size-count)
  //   << "\nNew target vel: [" << out.X() << "; " << out.Y() << "; " << out.Z() << "]"
  //   << "\n------------------------\n"
  // );

  return out;
}
