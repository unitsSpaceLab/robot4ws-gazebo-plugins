/**
* @author Simone Cottiga
* @email: simone.cottiga@phd.units.it
* @email: smcotti@gmail.com
**/

#include "../include/apply_single_wheel_terramechanic_model.h"


using namespace gazebo;


ApplySingleWheelTerramechanicModel::ApplySingleWheelTerramechanicModel() : ModelPlugin() 
{
  ROS_INFO("Starting apply_wheels_terramechanics_model plugin");
}


ApplySingleWheelTerramechanicModel::~ApplySingleWheelTerramechanicModel()
{
  // Safely disconnect update connection
  if (this->updateConnection) {
      this->updateConnection.reset();
  }

  // Safely disconnect contact subscriber
  if (this->dummy_contact_sub) {
      this->dummy_contact_sub.reset();
  }

  if (this->_ros_node) {
      this->_ros_node->shutdown();
      delete this->_ros_node;
      this->_ros_node = nullptr;
  }

  ROS_INFO("apply_wheels_terramechanics_model plugin shutted down");
}


void ApplySingleWheelTerramechanicModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO("Loading apply_wheels_terramechanics_model plugin...");

  this -> model = _model;
  this -> sdf = _sdf;

  this -> initializePluginParam();

  this -> initializeLinks();

  if (this -> publish_results)
  {this -> initializeROSelements();}

  this -> contact_manager = this->model->GetWorld()->Physics()->GetContactManager();

// START this part might be needed in order to use the gazebo contacts, in case...
  // there is nothing else listening to them (ex. contact sensors or client->View->Contacts activated)
  this -> dummy_contact_node = transport::NodePtr(new transport::Node());
  this -> dummy_contact_node -> Init();
  this -> dummy_contact_sub = this -> dummy_contact_node -> Subscribe("~/physics/contacts", &ApplySingleWheelTerramechanicModel::dummy_contact_callback, this);
// END this part might be needed in order to use the gazebo contacts, in case...

  if (this->options.use_compact_model)
  {
    this -> updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ApplySingleWheelTerramechanicModel::OnUpdateCompact, this));
  } else{
    this -> updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ApplySingleWheelTerramechanicModel::OnUpdate, this));
  }

  ROS_INFO_STREAM("apply_wheel_terramechanic_model plugin successfully loaded for [" << this->link_name << "]");
}


void ApplySingleWheelTerramechanicModel::OnUpdate(void)
{
  /// Run terramechanics model
  // TO DO: the model breaks (unrealistic behaviour) when (slip ratio < -1), i.e. (sign(v_x)*sign(omega)=-1 && abs(v_x)>abs(omega*r))

  // check contact and eventually update soil_params
/*   if (! this -> checkContacts())
  {
    // if the link has no contacts, skip
    if(this->debug){ROS_INFO_STREAM("DB [" << this->link_name << "]: No contact found");}
    return;
  } */

  // set wheel frame as contact frame, always apply force (if wheel is moving)
  this -> contact_frame_rot = this -> link_steer -> WorldPose().Rot();
  // rotate frames xy plane such that x axis points forward 
  if (this->link_name == "Archimede_br_wheel_link" || this->link_name == "Archimede_fr_wheel_link")
  {this -> contact_frame_rot = this -> contact_frame_rot * ignition::math::Quaterniond(0,0,M_PI);}
  this -> contact_frame_rot.Normalize();

  // manually set soil params, not needed here if checkContact function is used
  // this -> soil_params.name = "Sand_marsSim"; // [Soil_Direct_#90_sand, Sand_marsSim]
  this -> setSoilParams();

  // get wheel state, set wheel_states_params and compute slips
  this -> setWheelStateParams();

  // TODO: case wheel is static, decide what to do & condition (if needed)
  if (fabs(this->wheel_state_params.omega*this->wheel_params.r_s) < 0.02)
  {
    this -> link_wheel -> SetLinearVel(ignition::math::Vector3d(0,0,0));
    this -> link_wheel -> SetAngularVel(ignition::math::Vector3d(0,0,0));
    if(this->debug){ROS_INFO_STREAM("DB [" << this->link_name << "]: wheel is static");}
    return;
  }

  if(this->debug){ROS_INFO_STREAM("DB [" << this->link_name << "]: wheels rel xy vel: " << this->wheel_state_params.v_x << ", " << this->wheel_state_params.v_y
                                                            << "\n\t\t\t\t\t\tomega: " << this->wheel_state_params.omega << ", slip: " << this->wheel_state_params.s << ", beta: " << 180/M_PI*this->wheel_state_params.beta);}

  // set tuned_params
  this -> setTunedParams();

  // compute wheel load
  this -> computeWheelLoad();

  // find sinkage, if no sinkage is found return
  if (!this -> findSinkage()){return;}

  // compute forces/moments for found sinkage
  this -> computeForces();

  // apply forces/moments to the wheel
  this -> applyForce();

  // publish computed forces/moments (all of them)
  if (this -> publish_results)
  {this -> publishResults();}
}


void ApplySingleWheelTerramechanicModel::OnUpdateCompact(void)
{
  /// Run terramechanics compact model
  // TO DO: the model breaks (gazebo crash) when (slip ratio < -1), i.e. (sign(v_x)*sign(omega)=-1 && abs(v_x)>abs(omega*r))

  // check contact and eventually update soil_params
/*   if (! this -> checkContacts())
  {
    // if the link has no contacts, skip
    if(this->debug){ROS_INFO_STREAM("DB [" << this->link_name << "]: No contact found");}
    return;
  } */

  // set wheel frame as contact frame, always apply force (if wheel is moving)
  this -> contact_frame_rot = this -> link_steer -> WorldPose().Rot();
  // rotate frames xy plane such that x axis points forward
  if (this->link_name == "Archimede_br_wheel_link" || this->link_name == "Archimede_fr_wheel_link")
  {this -> contact_frame_rot = this -> contact_frame_rot * ignition::math::Quaterniond(0,0,M_PI);}
  this -> contact_frame_rot.Normalize();

  this -> wheel_params.r_s = this->wheel_params.r + 0.5 * this->wheel_params.h_g;

  // get wheel state, set wheel_states_params and compute slips
  this -> setWheelStateParams();

  // TODO: case wheel is static, decide what to do & condition (if needed, omega=0 breaks things for now)
  if (fabs(this->wheel_state_params.omega*this->wheel_params.r_s) < 0.02)
  {
    this -> link_wheel -> SetLinearVel(ignition::math::Vector3d(0,0,0));
    this -> link_wheel -> SetAngularVel(ignition::math::Vector3d(0,0,0));
    if(this->debug){ROS_INFO_STREAM("DB [" << this->link_name << "]: wheel is static");}
    return;
  }

  if(this->debug){ROS_INFO_STREAM("DB [" << this->link_name << "]: wheels rel xy vel: " << this->wheel_state_params.v_x << ", " << this->wheel_state_params.v_y
                                                            << "\n\t\t\t\t\t\tomega: " << this->wheel_state_params.omega << ", slip: " << this->wheel_state_params.s << ", beta: " << 180/M_PI*this->wheel_state_params.beta);}

  // manually set soil params
  // this -> soil_params.name = "Sand_marsSim"; // [Soil_Direct_#90_sand, Sand_marsSim]
  this -> setSoilParams();

  this -> setTunedParams();

  // compute wheel load
  this -> computeWheelLoad();

  // find sinkage iteratively equalizing computed F_z with wheel load
  double h_min=0, h_max=1.5*this->wheel_params.r_s, max_err=this->forces.W*pow(10,-3), F_z=0;
  double A, B, sigma_m, tau_xm, tau_ym;
  // binary search algorithm
  while (fabs(F_z - this->forces.W) > fabs(max_err))
  {
    double h_0 = (h_max+h_min)/2;

    // simplified geometry
    double theta_f = acos(1 - h_0 / this->wheel_params.r);
    double theta_r = 0;
    double theta_m = theta_f / 2;

    // normal stress
    sigma_m = this->soil_params.k * pow(this->wheel_params.r * (cos(theta_m) - cos(theta_f)),this->soil_params.n);

    // shear stress
    double j_xm = this->wheel_params.r * (theta_f - theta_m - (1 - this->wheel_state_params.s) * (sin(theta_f) - sin(theta_m)));
    double j_ym = this->wheel_params.r * (1 - this->wheel_state_params.s) * (theta_f - theta_m) * tan(fabs(this->wheel_state_params.beta));
    tau_xm = (this->soil_params.c + sigma_m * tan(this->soil_params.phi)) * (1 - exp(-j_xm / this->soil_params.K));
    tau_ym = (this->soil_params.c + sigma_m * tan(this->soil_params.phi)) * (1 - exp(-j_ym / this->soil_params.K));

    // Force coefficients
    A = (cos(theta_m) - cos(theta_r)) / (theta_m - theta_r) + (cos(theta_m) - cos(theta_f)) / (theta_f - theta_m);
    B = (sin(theta_m) - sin(theta_r)) / (theta_m - theta_r) + (sin(theta_m) - sin(theta_f)) / (theta_f - theta_m);

    // Normal force
    F_z = this->wheel_params.b * (this->wheel_params.r * sigma_m * A + this->wheel_params.r_s * tau_xm * B);

    if (F_z < this->forces.W)
    {h_min = h_0;}
    else
    {h_max = h_0;}
    if ((h_max-h_min) < pow(10,-6))
    {
      ROS_ERROR_STREAM("Sinkage not found for wheel [" << this->link_name << "]: expected Fz = " << this->forces.W << ", computed Fz = " << F_z << ", h_min_max = [" << h_min << " - " << h_max << "]");
      return;
    }
    this -> terramechanics_params.h_0 = h_0;
    this -> terramechanics_params.theta_f = theta_f;
    this -> terramechanics_params.theta_r = theta_r;
    this -> terramechanics_params.theta_m = theta_m;
  }
  if(this->debug){ROS_INFO_STREAM("DB [" << this->link_name << "]: Sinkage found: " << this->terramechanics_params.h_0);}

  double C = (this->terramechanics_params.theta_f - this->terramechanics_params.theta_r) / 2;

  // compute forces/moments
  this -> forces.F_x = this->wheel_params.b * (this->wheel_params.r_s * tau_xm * A - this->wheel_params.r * sigma_m * B);
  this -> forces.F_y = std::copysign(this->wheel_params.b * this->wheel_params.r_s * tau_ym * C, - this->wheel_state_params.beta); // bulldozing neglected
  this -> forces.F_z = F_z;
  this -> forces.M_x = this->forces.F_y * this->wheel_params.r_s;
  this -> forces.M_y = - this->wheel_params.b * pow(this->wheel_params.r_s, 2) * tau_xm * C;
  this -> forces.M_z = this->forces.F_y * this->wheel_params.r_s * sin(this->terramechanics_params.theta_m);

  // apply forces/moments to the wheel
  this -> applyForce();

  // publish computed forces/moments (all of them)
  if (this -> publish_results)
  {this -> publishResults();}
}


void ApplySingleWheelTerramechanicModel::applyForce(void)
{
  // case forces are in contact frame
  // contact frame: force -> z=0(keep gazebo's), xy = model's
  ignition::math::Vector3d force_to_add_contact(this->forces.F_x, this->forces.F_y, 0);
  // M_y = M_z = 0, they are drive and steer joints resistent moments, but do not influence the rover movement
  ignition::math::Vector3d torque_to_add_contact(this->forces.M_x, 0, 0);
  // ignition::math::Vector3d torque_to_add_contact(this->forces.M_x, this->forces.M_y, this->forces.M_z);

  // transform to world frame
  ignition::math::Vector3d force_to_add = this->contact_frame_rot.RotateVector(force_to_add_contact);
  ignition::math::Vector3d torque_to_add = this->contact_frame_rot.RotateVector(torque_to_add_contact);

  // apply force/torque in world frame to the wheel
  this -> link_wheel -> AddForce(force_to_add);  // force in world frame
  this -> link_wheel -> AddTorque(torque_to_add);  // torque in world frame

  if(this->debug){ROS_INFO_STREAM("DB [" << this->link_name << "]: applied forces: " << force_to_add.X() << " - " << force_to_add.Y() << " - " << force_to_add.Z()
                          << "\n\t\t\t\t\t\t\t\t\t\ttorques: " << torque_to_add.X() << " - " << torque_to_add.Y() << " - " << torque_to_add.Z());}
}


void ApplySingleWheelTerramechanicModel::initializePluginParam(void)
{
  if (this -> sdf -> HasElement("link_name"))
  {
    this -> link_name = this -> sdf -> GetElement("link_name") -> Get<std::string>();
  }

  if (this -> sdf -> HasElement("use_compact_model"))
  {this -> options.use_compact_model = this -> sdf -> GetElement("use_compact_model") -> Get<bool>();}
  else
  {this -> options.use_compact_model = true;}

  this -> debug = false; // print some debug stuffs in rosout

  this -> publish_results = true; // true: publish the computed forces/moments in the ros_pub_results_topic_name topic
}


void ApplySingleWheelTerramechanicModel::initializeLinks(void)
{
  this -> link_wheel = this -> model -> GetLink(this->link_name);
  if (!this->link_wheel)
  {
    ROS_ERROR_STREAM("Link [" << this->link_name << "] not found! Calling apply_wheel_terramechanic_model plugin Destructor...");
    this -> ~ApplySingleWheelTerramechanicModel();
  }

  this -> link_steer = this -> link_wheel -> GetParentJointsLinks()[0];
  if (!this->link_steer)
  {
    ROS_ERROR_STREAM("Steer link for [" << this->link_name << "] not found! Calling apply_wheel_terramechanic_model plugin Destructor...");
    this -> ~ApplySingleWheelTerramechanicModel();
  }

  this -> link_collision_name = this -> link_name + "_collision";
  physics::CollisionPtr link_collision = this -> link_wheel -> GetCollision(this->link_collision_name);
  this -> link_collision_name = link_collision -> GetName();

  // set the link friction coefficients to zero
  auto link_friction =  link_collision -> GetSurface() -> FrictionPyramid();
  link_friction -> SetMuPrimary(0);
  link_friction -> SetMuSecondary(0);
  link_friction -> SetMuTorsion(0);

  // link_collision -> GetSurface() -> collideWithoutContact = true;

  this -> setWheelParams();

  // get data for static load computation
  this -> world_gravity = fabs(this->model->GetWorld()->Gravity().Z());
  ignition::math::Vector3d wheel_pos_in_model = this->link_wheel->RelativePose().Pos();
  this -> link_mass = this->link_wheel->GetInertial()->Mass() + this->link_steer->GetInertial()->Mass();
  this->rover_dimensions[0] = 2 * fabs(wheel_pos_in_model.X());
  this->rover_dimensions[1] = 2 * fabs(wheel_pos_in_model.Y());
  if (this->link_name == "Archimede_br_wheel_link") {this->f_signs[0] = -1; this->f_signs[1] = -1;}
  else if (this->link_name == "Archimede_fr_wheel_link") {this->f_signs[0] = 1; this->f_signs[1] = -1;}
  else if (this->link_name == "Archimede_bl_wheel_link") {this->f_signs[0] = -1; this->f_signs[1] = 1;}
  else if (this->link_name == "Archimede_fl_wheel_link") {this->f_signs[0] = 1; this->f_signs[1] = 1;}
  else {
    ROS_ERROR_STREAM("Unrecognized link [" << this->link_name << "]! Calling apply_wheel_terramechanic_model plugin Destructor...");
    this -> ~ApplySingleWheelTerramechanicModel();
  }

  auto model_links = this->model->GetLinks();
  for (auto link : model_links)
  {
    if (link->GetName().find("wheel") != std::string::npos || link->GetName().find("steer") != std::string::npos) {continue;}

    double mass  = link->GetInertial()->Mass();
    if (mass == 0) {continue;}

    ignition::math::Vector3d wheel_to_link = link->RelativePose().Pos() - wheel_pos_in_model;

    std::vector<double> values = {mass, fabs(wheel_to_link.X()), fabs(wheel_to_link.Y()), fabs(wheel_to_link.Z())};

    this -> other_masses.push_back(values);
  }
}


void ApplySingleWheelTerramechanicModel::initializeROSelements(void)
{
  this -> _ros_node = new ros::NodeHandle();

  this -> ros_pub_results_topic_name = "terramechanic_forces";
  this -> ros_pub_results = this -> _ros_node -> advertise<robot4ws_msgs::Vector3Array>(this -> ros_pub_results_topic_name,100);

  this -> ros_pub_intermediate_values_topic_name = "terramechanic_intermediate_values";
  this -> ros_pub_intermediate_values = this -> _ros_node -> advertise<robot4ws_msgs::Float64NamedArray>(this -> ros_pub_intermediate_values_topic_name,100);

  this -> resultMsg.names.resize(4);
  this -> resultMsg.names[0] = this -> link_name + "::F_contact";
  this -> resultMsg.names[1] = this -> link_name + "::M_contact";
  this -> resultMsg.names[2] = this -> link_name + "::F_world";
  this -> resultMsg.names[3] = this -> link_name + "::M_world";
  this -> resultMsg.vectors.resize(4);

  std::vector<std::string> names = {"omega", "relative_v_x", "relative_v_y", "slip_ratio", "slip_angle", "wheel_load", "sinkage"};
  this -> intermediateValuesMsg.names.resize(names.size());
  this -> intermediateValuesMsg.data.resize(names.size());
  for (int i = 0; i < names.size(); i++)
  {this -> intermediateValuesMsg.names[i] = names[i];}
  this -> intermediateValuesMsg.header.frame_id = this -> link_name;
}


bool ApplySingleWheelTerramechanicModel::checkContacts(void)
{
  bool out = false;
  std::string contact_name;
  physics::Contact *contact_i;

  // Get link contacts (if any)
  for (int i = 0; i < this->contact_manager->GetContactCount(); i++)
  {
    contact_i = this -> contact_manager -> GetContact(i);
    // if(this->debug){ROS_INFO_STREAM("Contact " << i << ": " << contact_i->collision1->GetName() << "  -  " << contact_i->collision2->GetName() << "\n");}

    // skip collisions within the rover and between other models
    if ((contact_i->collision1->GetModel()->GetName()==this->model->GetName()) == (contact_i->collision2->GetModel()->GetName()==this->model->GetName()))
    {continue;}

    if (contact_i->collision1->GetName()==this->link_collision_name)
    {
      out = true;
      contact_name = contact_i->collision2->GetName();
      break;
    }
    else if (contact_i->collision2->GetName()==this->link_collision_name)
    {
      out = true;
      contact_name = contact_i->collision1->GetName();
      break;
    }
  }

  if (out)
  {
    // get contact normal
    double rx=0,ry=0,rz=0;
    int count = contact_i->count;
    for (int i = 0; i < count; i++)
    {
      ignition::math::Vector3d normal_i = contact_i->normals[i];
      rx += normal_i.X();
      ry += normal_i.Y();
      rz += normal_i.Z();
    }
    // get steer frame
    ignition::math::Pose3d steer_pose = this -> link_steer -> WorldPose();
    ignition::math::Vector3d steer_x_axis = steer_pose.Rot().XAxis().Normalize();

    // define contact frame: z axis along contact normal, x is the projection of steer x axis (wheel radial dir), y 3rd with right hand rule
    ignition::math::Vector3d z_dir = ignition::math::Vector3d(rx/count, ry/count, rz/count).Normalize();
    ignition::math::Vector3d x_dir = (steer_x_axis - z_dir * steer_x_axis.Dot(z_dir)).Normalize();
    ignition::math::Vector3d y_dir = z_dir.Cross(x_dir);
    ignition::math::Matrix3d contact_rot_mat;
    contact_rot_mat.Axes(x_dir,y_dir,z_dir);

    ignition::math::Matrix4d temp_mat;
    temp_mat = contact_rot_mat;

    this -> contact_frame_rot = temp_mat.Rotation();
    this -> contact_frame_rot.Normalize();
    // rotate frames xy plane such that x axis points forward 
    if (this->link_name == "Archimede_br_wheel_link" || this->link_name == "Archimede_fr_wheel_link")
    {
      this -> contact_frame_rot = this -> contact_frame_rot * ignition::math::Quaterniond(0,0,M_PI);
      this -> contact_frame_rot.Normalize();
    }

    // update soil_params if soil has changed
    if (contact_name != this->prev_contact_name)
    {
      this -> prev_contact_name = contact_name;
      this -> setSoilParams();
    }
  }

  return out;
}


void ApplySingleWheelTerramechanicModel::setWheelParams(void)
{
  // set wheel params (constants)
  this -> wheel_params.type = "grousers"; // smooth/grousers
  this -> wheel_params.r = 0.085;
  this -> wheel_params.b = 0.09;
  if (this->wheel_params.type == "smooth")
  {
    this -> wheel_params.h_g = 0;
    this -> wheel_params.mu = 1;
  } else{
    if (this->wheel_params.type != "grousers")
    {
      ROS_WARN("Unrecognized wheel type, defaulting to [grousers]...");
      this -> wheel_params.type = "grousers";
    }
    this -> wheel_params.h_g = 0.01;
    this -> wheel_params.mu = 0.5;
  }
  this -> wheel_params.r_s = wheel_params.r + wheel_params.h_g;
}


void ApplySingleWheelTerramechanicModel::setSoilParams(void)
{
  // this -> soil_params.name = "Soil_Direct_#90_sand";

  std::string terrain_below_name;
  double terrain_below_distance; // unused
  this -> link_wheel -> GetNearestEntityBelow(terrain_below_distance, terrain_below_name); // this takes the collision name

  // TODO: the mapping could be done in a separate file (.yaml or something) instead of here, it depends on world models
  // map model-soil type for khalid's world
  if (terrain_below_name.find("terrain_blue") != std::string::npos)
  {this -> soil_params.name = "Soil_Direct_#90_sand";}
  else if (terrain_below_name.find("terrain_red") != std::string::npos)
  {this -> soil_params.name = "Sand_marsSim";}
  else if (terrain_below_name.find("terrain_green") != std::string::npos)
  {this -> soil_params.name = "Clay";}
  else if (terrain_below_name.find("terrain_yellow") != std::string::npos)
  {this -> soil_params.name = "Dry_sand";}
  else // default
  {this -> soil_params.name = "Sand_marsSim";}

  if (this -> soil_params.name == "Soil_Direct_#90_sand")
  { // from Pavlov (2024)
    this -> soil_params.k = 8000 *pow(10,3);
    this -> soil_params.phi = 29 *M_PI/180;
    this -> soil_params.rho = 13.03 *pow(10,3);
    this -> soil_params.c = 1.0 *pow(10,3);
    this -> soil_params.K = 0.021;
    // n0, n1, n2 are assigned later in setTunedParams
  }
  else if (this->soil_params.name == "Clay")
  { // from Ding et al. (2015), table I
    this->soil_params.k_c = 13.19 * pow(10,3);
    this->soil_params.k_phi = 692.15 * pow(10,3);
    this->soil_params.k = (this->soil_params.k_c / this->wheel_params.b + this->soil_params.k_phi);
    this->soil_params.phi = 13 * M_PI/180;
    this->soil_params.c = 4.14 * pow(10,3);
    this->soil_params.K = 6 * pow(10,-3);

    this->soil_params.n = 0.5;

    this -> tuned_params.n0 = NAN;
    this -> tuned_params.n1 = NAN;
    this -> tuned_params.n2 = NAN;
  }
  else if (this->soil_params.name == "Dry_sand")
  { // from Ding et al. (2015), table I
    this->soil_params.k_c = 0.99 * pow(10,3);
    this->soil_params.k_phi = 1528.43 * pow(10,3);
    this->soil_params.k = (this->soil_params.k_c / this->wheel_params.b + this->soil_params.k_phi);
    this->soil_params.phi = 28 * M_PI/180;
    this->soil_params.c = 1.04 * pow(10,3);
    this->soil_params.K = 10 * pow(10,-3); // [10-25]*1e3, 10: firm sand - 25: loose sand

    this->soil_params.n = 1.10;

    this -> tuned_params.n0 = NAN;
    this -> tuned_params.n1 = NAN;
    this -> tuned_params.n2 = NAN;
  }
  else if (this->soil_params.name == "Lunar_soil")
  { // from Ding et al. (2015), table I
    this->soil_params.k_c = 1.4 * pow(10,3);
    this->soil_params.k_phi = 820 * pow(10,3);
    this->soil_params.k = (this->soil_params.k_c / this->wheel_params.b + this->soil_params.k_phi);
    this->soil_params.phi = 35 * M_PI/180; // [25-50]*pi/180
    this->soil_params.c = 0.17 * pow(10,3); // [0.1-2.7]*1e3
    this->soil_params.K = 18 * pow(10,-3);

    this->soil_params.n = 1.0;

    this -> tuned_params.n0 = NAN;
    this -> tuned_params.n1 = NAN;
    this -> tuned_params.n2 = NAN;
  }
  else if (this->soil_params.name == "HIT-LSS1")
  { // from Ding et al. (2015), table I
    this->soil_params.k_c = 15.6 * pow(10,3);
    this->soil_params.k_phi = 2407.4 * pow(10,3);
    this->soil_params.k = (this->soil_params.k_c / this->wheel_params.b + this->soil_params.k_phi);
    this->soil_params.phi = 31.9 * M_PI/180;
    this->soil_params.c = 0.25 * pow(10,3);
    this->soil_params.K = 10 * pow(10,-3); // [9.7-13.1]*1e-3

    this->soil_params.n = 1.10;

    this -> tuned_params.n0 = NAN;
    this -> tuned_params.n1 = NAN;
    this -> tuned_params.n2 = NAN;
  }
  else {
    if (!(this->soil_params.name == "Sand_marsSim"))
    {ROS_WARN("Unrecognized soil name! Default it to [Sand_marsSim] soil...");}
    // from Zhou et al. 2023 (MarsSim...) table2
    this -> soil_params.k_c = 13.6 * pow(10,3);
    this -> soil_params.k_phi = 2259.1 *pow(10,3);
    this -> soil_params.k = (this->soil_params.k_c / this->wheel_params.b + this->soil_params.k_phi);
    this -> soil_params.c = 0.4623 *pow(10,3);
    this -> soil_params.phi = 35 *M_PI/180;
    this -> soil_params.K = 0.015;

    this -> tuned_params.n0 = 0.92;
    this -> tuned_params.n1 = 0.5;
    this -> tuned_params.n2 = this -> tuned_params.n1;
  }
  this -> soil_params.X_c = 45*M_PI/180 - this->soil_params.phi/2;
}


void ApplySingleWheelTerramechanicModel::setWheelStateParams(void)
{
  // wheel angular drive velocity
  this -> wheel_state_params.omega = this -> link_wheel -> RelativeAngularVel().Y();

  // change sign where necessary so that omega > 0 for forward movement
  if (this->link_name == "Archimede_br_wheel_link" || this->link_name == "Archimede_fr_wheel_link")
  {this -> wheel_state_params.omega *= -1;}

  // if omega < 0 (backward movement), change its sign and rotate contact frame so it's the same as moving forward in the reverse direction
  if (this -> wheel_state_params.omega < 0)
  {
    this -> wheel_state_params.omega *= -1;
    this -> contact_frame_rot = this -> contact_frame_rot * ignition::math::Quaterniond(0,0,M_PI);
    this -> contact_frame_rot.Normalize();
  }

  // wheel velocity in contact frame
  ignition::math::Vector3d link_relative_vel = this->contact_frame_rot.RotateVectorReverse(this->link_steer->WorldLinearVel());
  this -> wheel_state_params.v_x = link_relative_vel.X();
  this -> wheel_state_params.v_y = link_relative_vel.Y();

  this -> wheel_state_params.v = sqrt(pow(this->wheel_state_params.v_x,2) + pow(this->wheel_state_params.v_y,2));
  this -> wheel_state_params.beta = atan(this->wheel_state_params.v_y / this->wheel_state_params.v_x);

  if (fabs(this->wheel_state_params.omega*this->wheel_params.r_s) <= pow(10,-4) && fabs(this->wheel_state_params.v_x) <= pow(10,-4))
  {
    // case 0/0
    this -> wheel_state_params.s = 0;
  }
  else if (fabs(this->wheel_params.r_s*this->wheel_state_params.omega) >= fabs(this->wheel_state_params.v_x))
  {
    // slip
    this -> wheel_state_params.s = (this->wheel_params.r_s*this->wheel_state_params.omega - this->wheel_state_params.v_x) / (this->wheel_params.r_s*this->wheel_state_params.omega);
  } else{
    // skid
    this -> wheel_state_params.s = (this->wheel_params.r_s*this->wheel_state_params.omega - this->wheel_state_params.v_x) / (this->wheel_state_params.v_x);
  }
}


void ApplySingleWheelTerramechanicModel::setTunedParams(void)
{
  double beta = fabs(this -> wheel_state_params.beta);
  if (this -> wheel_params.type == "smooth" && this->soil_params.name == "Soil_Direct_#90_sand")
  {
    // coeff interpolated from Pavlov data (wheels won't be the same), not used in simplified geometry
    this -> tuned_params.a0 = -0.0539*pow(beta,3) -0.0227*pow(beta,2) +0.6294*beta +0.1674;
    this -> tuned_params.a1 = 0.0156*pow(beta,3) +0.3414*pow(beta,2) -1.0039*beta +0.7632;
    this -> tuned_params.b0 = -0.4386*pow(beta,4) +0.8719*pow(beta,3) -0.5565*pow(beta,2) +0.0905*beta -0.4753;
    this -> tuned_params.b1 = 0.4178*pow(beta,3) -0.8113*pow(beta,2) +0.2433*beta -0.0044;

    this -> tuned_params.n0 = 1.46;
    this -> tuned_params.n1 = 0.01;
    this -> tuned_params.n2 = 0.55;
  }
  else if (this->wheel_params.type == "grousers" && this->soil_params.name == "Soil_Direct_#90_sand")
  {
    this -> tuned_params.a0 = 0.7450*pow(beta,4) -2.4800*pow(beta,3) +2.6033*pow(beta,2) -0.4332*beta +0.2716;
    this -> tuned_params.a1 = -0.6876*pow(beta,4) +2.3687*pow(beta,3) -2.5096*pow(beta,2) +0.3618*beta +0.6761;
    this -> tuned_params.b0 = -0.3198*pow(beta,3) +0.1680*pow(beta,2) +0.3219*beta -0.6372;
    this -> tuned_params.b1 = 0.1387*pow(beta,3) +0.3971*pow(beta,2) -0.8644*beta -0.1921;

    this -> tuned_params.n0 = 1.46;
    this -> tuned_params.n1 = 0.01;
    this -> tuned_params.n2 = 0.74;
  }

  this -> tuned_params.d0 = 1;
  this -> tuned_params.d1 = 0.5;

  // case n is already set for the soil
  if (std::isnan(this->tuned_params.n0) || std::isnan(this->tuned_params.n1) || std::isnan(this->tuned_params.n2))
  {return;}

  if (this->wheel_state_params.s >= 0)
  {this -> soil_params.n = this->tuned_params.n0 + this->tuned_params.n1 * this->wheel_state_params.s;}
  else
  {this -> soil_params.n = this->tuned_params.n0 - this->tuned_params.n2 * this->wheel_state_params.s;}
}


bool ApplySingleWheelTerramechanicModel::findSinkage(void)
{
  // find sinkage iteratively equalizing computed F_z with wheel load
  double h_min=0, h_max=1.5*this->wheel_params.r_s, max_err=this->forces.W*pow(10,-3), F_z=0;

  // binary search algorithm
  while (fabs(F_z - this->forces.W) > fabs(max_err))
  {
    this -> terramechanics_params.h_0 = (h_max+h_min)/2;

    this -> computeContactGeometry();

    this -> computeStresses();

    this -> computeForces(&F_z);

    if (F_z < this->forces.W)
    {
      h_min = this -> terramechanics_params.h_0;
    }
    else
    {
      h_max = this -> terramechanics_params.h_0;
    }
    if ((h_max-h_min) < pow(10,-6))
    {
      ROS_ERROR_STREAM("Sinkage not found for wheel [" << this->link_name << "]: expected Fz = " << this->forces.W << ", computed Fz = " << F_z << ", h_min_max = [" << h_min << " - " << h_max << "]");
      return false;
    }
  }
  if(this->debug){ROS_INFO_STREAM("DB [" << this->link_name << "]: Sinkage found: " << this->terramechanics_params.h_0);}
  return true;
}


void ApplySingleWheelTerramechanicModel::computeContactGeometry(void)
{
  // unpack params
  double h_0 = this->terramechanics_params.h_0;
  double r_s = this->wheel_params.r_s;
  double s = this->wheel_state_params.s;
  double a0 = this->tuned_params.a0;
  double a1 = this->tuned_params.a1;
  double b0 = this->tuned_params.b0;
  double b1 = this->tuned_params.b1;
  int rim_pts = this->options.rim_pts;

  double theta_f, theta_r, theta_m, theta[rim_pts], h[rim_pts];

  theta_f = acos(1 - h_0/r_s);

/*   // complex geometry
  // for s>1, theta_m>theta_f breaks things
  if (s >= 0)
  {
    theta_m = theta_f * (a0 + a1*s);
    theta_r = theta_f * (b0 + b1*s);
  } else{
    theta_m = theta_f * a0;
    theta_r = theta_f * b0;
  } */

  // simplified geometry
  theta_r = 0;
  theta_m = (theta_f+theta_r) / 2;

  double d_theta = (theta_f-theta_r) / (rim_pts-1);
  for (int i = 0; i < rim_pts; i++)
  {
    theta[i] = theta_r + i*d_theta;

    h[i] = r_s * (cos(theta[i]) - cos(theta_f));
  }

  // update params
  this->terramechanics_params.theta_f = theta_f;
  this->terramechanics_params.theta_r = theta_r;
  this->terramechanics_params.theta_m = theta_m;
  std::memcpy(this->terramechanics_params.theta, theta, rim_pts * sizeof(double));
  std::memcpy(this->terramechanics_params.h, h, rim_pts * sizeof(double));
}


void ApplySingleWheelTerramechanicModel::computeStresses(void)
{
  // unpack params
  int rim_pts = this -> options.rim_pts;
  double k_c = this -> soil_params.k_c;
  double k_phi = this -> soil_params.k_phi;
  double k = this -> soil_params.k;
  double c = this -> soil_params.c;
  double phi = this -> soil_params.phi;
  double K = this -> soil_params.K;
  double n = this -> soil_params.n;
  double r = this -> wheel_params.r;
  double r_s = this -> wheel_params.r_s;
  double b = this -> wheel_params.b;
  double mu = this -> wheel_params.mu;
  double d0 = this -> tuned_params.d0;
  double d1 = this -> tuned_params.d1;
  double v_y = this -> wheel_state_params.v_y;
  double omega = this -> wheel_state_params.omega;
  double s = this -> wheel_state_params.s;
  double theta_f = this -> terramechanics_params.theta_f;
  double theta_r = this -> terramechanics_params.theta_r;
  double theta_m = this -> terramechanics_params.theta_m;
  double theta[rim_pts];
  std::memcpy(theta, this->terramechanics_params.theta, rim_pts * sizeof(double));

  double theta_e[rim_pts], sigma[rim_pts], tau[rim_pts], tau_t[rim_pts], tau_l[rim_pts], j[rim_pts], j_t[rim_pts], j_l[rim_pts], v_jt[rim_pts], v_jl[rim_pts];

  double theta_0 = theta_f * (d0 + d1*s);

  std::ostringstream out_info; out_info << "DB [" << this->link_name << "]:";
  for (size_t i = 0; i < rim_pts; i++)
  {
    // normal stress
    if (theta[i] >= theta_m)
    {theta_e[i] = theta[i];}
    else
    {theta_e[i] = theta_f - (theta[i]-theta_r) * (theta_f-theta_m) / (theta_m-theta_r);}

    double sigma_r, sigma_rs;
    if (fabs(theta[i] - theta_f) < pow(10,-6))
    {
      // if (cos(theta_e[i])-cos(theta_f)) is slightly negative (when we should be in theta_f), pow gives nan values
      sigma_r = 0;
      sigma_rs = 0;
    } else{
      sigma_r = k * pow(r,n) * pow((cos(theta_e[i])-cos(theta_f)),n);
      sigma_rs = k * pow(r_s,n) * pow((cos(theta_e[i])-cos(theta_f)),n);
    }

    if(this->debug){out_info << "\n\ttheta_i = "<< theta[i]<<"\n\tsigma_rs: " << sigma_rs << " = " << k * pow(r_s,n) << " * pow(" << (cos(theta_e[i])-cos(theta_f))<<","<<n<<")";}

    sigma[i] = mu*sigma_rs + (1-mu)*sigma_r;

    // shear stress
    v_jl[i] = v_y;
    j_l[i] = (theta_f-theta[i]) * v_y / omega;
    if(this->debug){out_info <<"\n\tv_jl = " << v_jl[i] << ", j_l = " << j_l[i];}

    if (s >= 0)
    {
      v_jt[i] = omega * r_s * (1 - (1-s)*cos(theta[i]));
      j_t[i] = r_s * ((theta_f-theta[i]) - (1-s)*(sin(theta_f)-sin(theta[i])));
      if(this->debug){out_info << "\n\tv_jt1: " << v_jt[i] << "=" << omega << "*" <<r_s<< "* (1 - "<<1-s<<"*"<<cos(theta[i])<<")\n\tj_t1: " << j_t[i] << "=" << r_s <<"*"<< "(("<< theta_f<<"-"<<theta[i]<<") - ("<<1-s<<")*("<<sin(theta_f)<<"-"<<sin(theta[i])<<"))";}
    }
    else if (theta[i] >= theta_0){
      v_jt[i] = omega * r_s / (1+s) * ((sin(theta_f)-sin(theta_0)) / (theta_f-theta_0) - cos(theta[i]));
      j_t[i] = r_s / (1+s) * ((sin(theta_f)-sin(theta_0)) * (theta_f-theta[i]) / (theta_f-theta_0) - (sin(theta_f)-sin(theta[i])));
      if(this->debug){out_info << "\n\tv_jt2: " << v_jt[i] << "=" << omega << "*" <<r_s<< "* (("<<sin(theta_f)<<"-"<<sin(theta_0)<<") / ("<<theta_f<<"-"<<theta_0<<") - "<<cos(theta[i]) <<"/"<< (1+s)<< "\n\tj_t2: " << j_t[i] << "=" << r_s <<"/"<< (1+s) <<"* (("<<sin(theta_f)<<"-"<<sin(theta_0)<<") * ("<<theta_f<<"-"<<theta[i]<<") / ("<<theta_f<<"-"<<theta_0<<") - ("<<sin(theta_f)<<"-"<<sin(theta[i])<<"))";}
    }
    else{
      v_jt[i] = omega * r_s * (1 - cos(theta[i]) / (1+s));
      j_t[i] = r_s * ((theta_0-theta[i]) - (sin(theta_0)-sin(theta[i])) / (1+s));
      if(this->debug){out_info << "\n\tv_jt3: " << v_jt[i] << "=" << omega << "*" <<r_s<< "* ("<<1 - cos(theta[i])<< "/"<< (1+s)<<")\n\tj_t3: " << j_t[i] << "=" << r_s <<"*(("<<theta_0<<"-"<<theta[i]<<") - ("<<sin(theta_0)<<"-"<<sin(theta[i])<<") /"<< (1+s)<<")";}
    }

    j[i] = sqrt(pow(j_t[i],2) + pow(j_l[i],2));

    tau[i] = (c + sigma[i]*tan(phi)) * (1 - exp(-j[i]/K));
    tau_t[i] = tau[i] * v_jt[i] / sqrt(pow(v_jt[i],2) + pow(v_jl[i],2));
    tau_l[i] = tau[i] * v_jl[i] / sqrt(pow(v_jt[i],2) + pow(v_jl[i],2));
    if(this->debug){out_info << "\n\tsigma = " << sigma[i] << ", tau = " << tau[i];}
  }
  // if(this->debug){ROS_INFO_STREAM(out_info.str());}

  // update params
  std::memcpy(this -> terramechanics_params.theta_e, theta_e, rim_pts * sizeof(double));
  std::memcpy(this -> terramechanics_params.sigma, sigma, rim_pts * sizeof(double));
  std::memcpy(this -> terramechanics_params.tau, tau, rim_pts * sizeof(double));
  std::memcpy(this -> terramechanics_params.tau_t, tau_t, rim_pts * sizeof(double));
  std::memcpy(this -> terramechanics_params.tau_l, tau_l, rim_pts * sizeof(double));
  std::memcpy(this -> terramechanics_params.j, j, rim_pts * sizeof(double));
  std::memcpy(this -> terramechanics_params.j_t, j_t, rim_pts * sizeof(double));
  std::memcpy(this -> terramechanics_params.j_l, j_l, rim_pts * sizeof(double));
  std::memcpy(this -> terramechanics_params.v_jt, v_jt, rim_pts * sizeof(double));
  std::memcpy(this -> terramechanics_params.v_jl, v_jl, rim_pts * sizeof(double));
  this -> terramechanics_params.theta_0 = theta_0;
}


void ApplySingleWheelTerramechanicModel::computeForces(double* F_z_out)
{
  // if F_z_out is passed as an argument this only computes F_z and assignes it to F_z_out,
  // otherwise it computes all forces/moments and updates the forces params

  // unpack params
  int rim_pts = this -> options.rim_pts;
  std::string bulldozing_resistence = this -> options.bulldozing_resistence;
  double X_c = this -> soil_params.X_c;
  double c = this -> soil_params.c;
  double rho = this -> soil_params.rho;
  double phi = this -> soil_params.phi;
  double theta_f = this -> terramechanics_params.theta_f;
  double theta_r = this -> terramechanics_params.theta_r;
  double theta_m = this -> terramechanics_params.theta_m;
  double beta = this -> wheel_state_params.beta;
  double r_s = this -> wheel_params.r_s;
  double b = this -> wheel_params.b;
  double sigma[rim_pts], tau_l[rim_pts], tau_t[rim_pts], theta[rim_pts];
  std::memcpy(sigma, this -> terramechanics_params.sigma, rim_pts * sizeof(double));
  std::memcpy(tau_l, this -> terramechanics_params.tau_l, rim_pts * sizeof(double));
  std::memcpy(tau_t, this -> terramechanics_params.tau_t, rim_pts * sizeof(double));
  std::memcpy(theta, this -> terramechanics_params.theta, rim_pts * sizeof(double));

  double d_theta = (theta_f-theta_r) / (rim_pts - 1);
  double F_z = 0;

  for (size_t i = 1; i < rim_pts; i++)
  {
    F_z += r_s * b * d_theta * (tau_t[i] * sin(theta[i]) + sigma[i] * cos(theta[i]));
  }
  if (F_z_out)
  {
    *F_z_out = F_z;
    return;
  }

  double R_b[rim_pts], F_x=0, F_y=0, M_x=0, M_y=0, M_z=0;

  if (bulldozing_resistence == "Ishigami"){
    double D1 = 1/tan(X_c) + tan(X_c + phi);
    double D2 = 1/tan(X_c) + pow((1/tan(X_c)),2) / (1/tan(phi));
    for (size_t i = 1; i < rim_pts; i++)
    {
      double hh = r_s * (cos(theta[i])-cos(theta_f));
      R_b[i] = D1 * (c*hh + 0.5*D2*rho*pow(hh,2));

      // negative sign so that F_y has opposite direction to wheel lateral velocity
      F_y -= d_theta * (r_s * b * tau_l[i] + R_b[i] * sin(beta) * (r_s - hh * cos(theta[i])));

      F_x += r_s * b * d_theta * (tau_t[i] * cos(theta[i]) - sigma[i] * sin(theta[i]));

      M_y -= pow(r_s,2) * b * d_theta * tau_t[i];
    }
  } else if (bulldozing_resistence == "Pavlov"){
    for (size_t i = 1; i < rim_pts; i++)
    {
      double h_b = r_s * ((sin(theta[i])-sin(theta_r)) * (cos(theta_r)-cos(theta_f)) / (sin(theta_f)-sin(theta_r)) - (cos(theta_r)-cos(theta[i])));
      R_b[i] = rho/2 * pow(h_b,2) * pow((1/tan(X_c)),2) * (1 + 0.5 / tan(X_c) * tan(phi)) + 2 * h_b * c / tan(X_c);

      // negative sign so that F_y has opposite direction to wheel lateral velocity
      F_y -= r_s * d_theta * (b * tau_l[i] + R_b[i] * sin(beta) * cos(theta[i]));

      F_x += r_s * b * d_theta * (tau_t[i] * cos(theta[i]) - sigma[i] * sin(theta[i]));

      M_y -= pow(r_s,2) * b * d_theta * tau_t[i];
    }
  } else{
    if (bulldozing_resistence != "neglect"){
      ROS_WARN("bulldozing_resistence option not recognised. Neglecting sidewall bulldozing force...");
    }
    for (size_t i = 1; i < rim_pts; i++)
    {
      // negative sign so that F_y has opposite direction to wheel lateral velocity
      F_y -= d_theta * (r_s * b * tau_l[i]);

      F_x += r_s * b * d_theta * (tau_t[i] * cos(theta[i]) - sigma[i] * sin(theta[i]));

      M_y -= pow(r_s,2) * b * d_theta * tau_t[i];
    }
  }

  M_x += F_y * r_s;
  M_z += F_y * r_s * sin(theta_m);

  // update params
  std::memcpy(this -> forces.R_b, R_b, rim_pts * sizeof(double));
  this -> forces.F_x = F_x;
  this -> forces.F_y = F_y;
  this -> forces.F_z = F_z;
  this -> forces.M_x = M_x;
  this -> forces.M_y = M_y;
  this -> forces.M_z = M_z;
}


void ApplySingleWheelTerramechanicModel::computeWheelLoad(void)
{
  // compute static load on the wheel and gravity component tangential to the ground according
  // to the rover orientation wrt to gravity direction
  // this does not account for dynamics effects (curves and acceleration/deceleration),
  // and always considers all 4 wheels in contact with the terrain

  ignition::math::Quaterniond rover_orient = this->model->WorldPose().Rot();

  // rover (and assumed terrain) inclinations around lateral & longitudinal axis
  double theta = -rover_orient.Pitch(); // positive -> forward is uphill
  double alpha = rover_orient.Roll(); // positive -> left is uphill

  double wheel_load = this->link_mass;

  for (std::vector<double> link_i : this->other_masses)
  {
    wheel_load += link_i[0] * (1 - (link_i[1] + this->f_signs[0]*link_i[3]*tan(theta))/this->rover_dimensions[0]) * (1 - (link_i[2] + this->f_signs[1]*link_i[3]*tan(alpha))/this->rover_dimensions[1]);
  }

  // wheel load along gravity direction (world negative z-axis)
  wheel_load *= this -> world_gravity;

  // load components in contact frame
  ignition::math::Vector3d wheel_load_contact = this->contact_frame_rot.RotateVectorReverse(ignition::math::Vector3d(0,0,-wheel_load));

  if (this->debug){ROS_INFO_STREAM("DB [" << this->link_name << "]: Load on wheel xyz:" << wheel_load_contact.X() << ", " << wheel_load_contact.Y() << ", " << wheel_load_contact.Z());}

  this -> forces.W = fabs(wheel_load_contact.Z());
}


void ApplySingleWheelTerramechanicModel::publishResults(void)
{
  // stamps
  this -> resultMsg.header.stamp = ros::Time::now();
  this -> intermediateValuesMsg.header.stamp = ros::Time::now();

  // fill forces msg
  // contact frame
  ignition::math::Vector3d force_contact(this->forces.F_x, this->forces.F_y, this->forces.F_z);
  ignition::math::Vector3d torque_contact(this->forces.M_x, this->forces.M_y, this->forces.M_z);

  // world frame
  ignition::math::Vector3d force_world = this->contact_frame_rot.RotateVector(force_contact);
  ignition::math::Vector3d torque_world = this->contact_frame_rot.RotateVector(torque_contact);

  this -> resultMsg.vectors[0].x = force_contact.X();
  this -> resultMsg.vectors[0].y = force_contact.Y();
  this -> resultMsg.vectors[0].z = force_contact.Z();
  this -> resultMsg.vectors[1].x = torque_contact.X();
  this -> resultMsg.vectors[1].y = torque_contact.Y();
  this -> resultMsg.vectors[1].z = torque_contact.Z();
  this -> resultMsg.vectors[2].x = force_world.X();
  this -> resultMsg.vectors[2].y = force_world.Y();
  this -> resultMsg.vectors[2].z = force_world.Z();
  this -> resultMsg.vectors[3].x = torque_world.X();
  this -> resultMsg.vectors[3].y = torque_world.Y();
  this -> resultMsg.vectors[3].z = torque_world.Z();

  // fill intermediate values msg
  this -> intermediateValuesMsg.data[0] = this -> wheel_state_params.omega;
  this -> intermediateValuesMsg.data[1] = this -> wheel_state_params.v_x;
  this -> intermediateValuesMsg.data[2] = this -> wheel_state_params.v_y;
  this -> intermediateValuesMsg.data[3] = this -> wheel_state_params.s;
  this -> intermediateValuesMsg.data[4] = this -> wheel_state_params.beta * 180/M_PI;
  this -> intermediateValuesMsg.data[5] = this -> forces.W;
  this -> intermediateValuesMsg.data[6] = this -> terramechanics_params.h_0;

  // publish
  this -> ros_pub_results.publish(this -> resultMsg);
  this -> ros_pub_intermediate_values.publish(this -> intermediateValuesMsg);
}

