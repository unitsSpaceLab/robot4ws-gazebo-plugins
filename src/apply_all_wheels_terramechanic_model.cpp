#include "../include/apply_all_wheels_terramechanic_model.h"
#include <omp.h>

using namespace gazebo;

// Constructor
ApplyAllWheelsTerramechanicModel::ApplyAllWheelsTerramechanicModel() : ModelPlugin() 
{
    ROS_INFO("Starting apply_all_wheels_terramechanics_model plugin");
    plugin_state = PluginState::UNINITIALIZED;
}

// Destructor
ApplyAllWheelsTerramechanicModel::~ApplyAllWheelsTerramechanicModel()
{
    plugin_state = PluginState::SHUTTING_DOWN;

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

    ROS_INFO("apply_all_wheels_terramechanics_model plugin shut down");
}

// Load function - called when plugin is loaded
void ApplyAllWheelsTerramechanicModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    plugin_state = PluginState::LOADING;
    ROS_INFO("Loading apply_all_wheels_terramechanics_model plugin...");

    this->model = _model;
    this->sdf = _sdf;

    this->initializePluginParam();
    this->initializeWheels();

    // // Cache common values for faster computation
    // for (int i = 0; i < num_wheels; i++) {
    //     wheels[i].cached_values.r_squared = pow(wheels[i].wheel_params.r, 2);
    //     wheels[i].cached_values.r_s_squared = pow(wheels[i].wheel_params.r_s, 2);
    //     if (!std::isnan(wheels[i].soil_params.phi)) {
    //         wheels[i].cached_values.sin_phi = sin(wheels[i].soil_params.phi);
    //         wheels[i].cached_values.cos_phi = cos(wheels[i].soil_params.phi);
    //         wheels[i].cached_values.tan_phi = tan(wheels[i].soil_params.phi);
    //     }
    // }

    if (this->publish_results) {
        this->initializeROSelements();
    }

    this->contact_manager = this->model->GetWorld()->Physics()->GetContactManager();

    // Initialize contact dummy subscriber
    this->dummy_contact_node = transport::NodePtr(new transport::Node());
    this->dummy_contact_node->Init();
    this->dummy_contact_sub = this->dummy_contact_node->Subscribe("~/physics/contacts", 
                             &ApplyAllWheelsTerramechanicModel::dummy_contact_callback, this);

    // Set the update callback
    if (this->options.use_compact_model) {
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ApplyAllWheelsTerramechanicModel::OnUpdateCompact, this));
    } else {
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&ApplyAllWheelsTerramechanicModel::OnUpdate, this));
    }

    plugin_state = PluginState::INITIALIZED;
    ROS_INFO("apply_all_wheels_terramechanics_model plugin successfully loaded");
}

// Initialize method - required by Gazebo ModelPlugin
void ApplyAllWheelsTerramechanicModel::Init()
{
    ROS_INFO("apply_all_wheels_terramechanics_model Init() called");

    // Check that wheels are properly initialized
    bool all_wheels_ready = true;
    for (int i = 0; i < num_wheels; i++) {
        if (!wheels[i].wheel_link || !wheels[i].steer_link) {
            all_wheels_ready = false;
            break;
        }
    }

    if (!all_wheels_ready) {
        ROS_ERROR("Not all wheels initialized properly in Init()");
    } else {
        plugin_state = PluginState::RUNNING;
    }

    // Set OpenMP threads to match number of wheels
    omp_set_num_threads(num_wheels);

    // Check OpenMP status
    int num_threads = 0;
    #pragma omp parallel
    {
        #pragma omp master
        {
            num_threads = omp_get_num_threads();
            ROS_INFO("OpenMP is enabled with %d threads", num_threads);
        }
    }
}

// Reset method - required by Gazebo ModelPlugin
void ApplyAllWheelsTerramechanicModel::Reset()
{
    ROS_INFO("apply_all_wheels_terramechanics_model Reset() called");

    // Reset any state that needs to be reset when simulation resets
    for (int i = 0; i < num_wheels; i++) {
        // Reset forces
        wheels[i].forces.F_x = 0;
        wheels[i].forces.F_y = 0;
        wheels[i].forces.F_z = 0;
        wheels[i].forces.M_x = 0;
        wheels[i].forces.M_y = 0;
        wheels[i].forces.M_z = 0;

        // Reset cached sinkage for warm start
        wheels[i].terramechanics_params.h_0 = 0;
    }

    // update_step_counter = 0;
}


// Initialize plugin parameters
void ApplyAllWheelsTerramechanicModel::initializePluginParam()
{
    if (this->sdf->HasElement("use_compact_model")) {
        this->options.use_compact_model = this->sdf->GetElement("use_compact_model")->Get<bool>();
    } else {
        this->options.use_compact_model = true;
    }

    // if (this->sdf->HasElement("bulldozing_resistance")) {
    //     this->options.bulldozing_resistence = this->sdf->GetElement("bulldozing_resistance")->Get<std::string>();
    // }

    // // if (this->sdf->HasElement("skip_update_steps")) {
    // //     this->options.skip_update_steps = this->sdf->GetElement("skip_update_steps")->Get<int>();
    // // } else {
    // //     this->options.skip_update_steps = 0; // Default: process every step
    // // }

    this->debug = false; // print some debug stuffs in rosout
    this->publish_results = true; // true: publish the computed forces/moments
}

// Initialize all wheel data
void ApplyAllWheelsTerramechanicModel::initializeWheels()
{
    // Initialize all wheels
    bool all_wheels_initialized = true;
    for (int i = 0; i < num_wheels; i++) {
        wheels[i].name = wheel_names[i];
        wheels[i].initVectors(this->options.rim_pts);
        all_wheels_initialized &= initializeWheel(i);
    }

    if (!all_wheels_initialized) {
        ROS_ERROR("Failed to initialize one or more wheels");
    }
}

// Initialize a single wheel
bool ApplyAllWheelsTerramechanicModel::initializeWheel(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];

    // Get wheel link
    wheel.wheel_link = this->model->GetLink(wheel.name);
    if (!wheel.wheel_link) {
        ROS_ERROR_STREAM("Link [" << wheel.name << "] not found!");
        return false;
    }

    // Get steer link
    wheel.steer_link = wheel.wheel_link->GetParentJointsLinks()[0];
    if (!wheel.steer_link) {
        ROS_ERROR_STREAM("Steer link for [" << wheel.name << "] not found!");
        return false;
    }

    // Get collision name
    wheel.collision_name = wheel.name + "_collision";
    physics::CollisionPtr link_collision = wheel.wheel_link->GetCollision(wheel.collision_name);
    wheel.collision_name = link_collision->GetName();

    // Set friction to zero
    auto link_friction = link_collision->GetSurface()->FrictionPyramid();
    link_friction->SetMuPrimary(0);
    link_friction->SetMuSecondary(0);
    link_friction->SetMuTorsion(0);

    // Set wheel parameters
    setWheelParams(wheel_idx);

    // Get data for static load computation
    this->world_gravity = fabs(this->model->GetWorld()->Gravity().Z());
    ignition::math::Vector3d wheel_pos_in_model = wheel.wheel_link->RelativePose().Pos();
    wheel.link_mass = wheel.wheel_link->GetInertial()->Mass() + 
                       wheel.steer_link->GetInertial()->Mass();

    // Set rover dimensions once
    if (wheel_idx == 0) {
        this->rover_dimensions[0] = 2 * fabs(wheel_pos_in_model.X());
        this->rover_dimensions[1] = 2 * fabs(wheel_pos_in_model.Y());

        // Collect other masses
        auto model_links = this->model->GetLinks();
        this->other_masses.clear();

        for (auto link : model_links) {
            if (link->GetName().find("wheel") != std::string::npos || 
                link->GetName().find("steer") != std::string::npos) {
                continue;
            }

            double mass = link->GetInertial()->Mass();
            if (mass == 0) {
                continue;
            }

            ignition::math::Vector3d wheel_to_link = link->RelativePose().Pos() - wheel_pos_in_model;
            std::vector<double> values = {
                mass, 
                fabs(wheel_to_link.X()), 
                fabs(wheel_to_link.Y()), 
                fabs(wheel_to_link.Z())
            };

            this->other_masses.push_back(values);
        }
    }

    // Set force signs based on wheel position
    if (wheel.name == "Archimede_br_wheel_link") {
        wheel.f_signs[0] = -1; 
        wheel.f_signs[1] = -1;
    } else if (wheel.name == "Archimede_fr_wheel_link") {
        wheel.f_signs[0] = 1; 
        wheel.f_signs[1] = -1;
    } else if (wheel.name == "Archimede_bl_wheel_link") {
        wheel.f_signs[0] = -1; 
        wheel.f_signs[1] = 1;
    } else if (wheel.name == "Archimede_fl_wheel_link") {
        wheel.f_signs[0] = 1; 
        wheel.f_signs[1] = 1;
    } else {
        ROS_ERROR_STREAM("Unrecognized wheel name: " << wheel.name);
        return false;
    }

    return true;
}

void ApplyAllWheelsTerramechanicModel::initializeROSelements()
{
    this->_ros_node = new ros::NodeHandle();
    this->ros_pub_results_topic_name = "terramechanic_forces";
    this->ros_pub_intermediate_values_topic_name = "terramechanic_intermediate_values";

    std::vector<std::string> intermediate_values_names = {"omega", "relative_v_x", "relative_v_y", "slip_ratio", "slip_angle", "wheel_load", "sinkage"};

    for (int i = 0; i < num_wheels; i++) {
        WheelData& wheel = wheels[i];

        // Create publishers
        wheel.ros_pub_results = this->_ros_node->advertise<robot4ws_msgs::Vector3Array>(
            this->ros_pub_results_topic_name, 100);
        wheel.ros_pub_intermediate_values = this->_ros_node->advertise<robot4ws_msgs::Float64NamedArray>(
            this->ros_pub_intermediate_values_topic_name, 100);

        // Initialize messages
        wheel.resultMsg.names.resize(4);
        wheel.resultMsg.names[0] = wheel.name + "::F_contact";
        wheel.resultMsg.names[1] = wheel.name + "::M_contact";
        wheel.resultMsg.names[2] = wheel.name + "::F_world";
        wheel.resultMsg.names[3] = wheel.name + "::M_world";
        wheel.resultMsg.vectors.resize(4);

        wheel.intermediateValuesMsg.names.resize(intermediate_values_names.size());
        wheel.intermediateValuesMsg.data.resize(intermediate_values_names.size());
        wheel.intermediateValuesMsg.header.frame_id = wheel.name;
        for (int i = 0; i < intermediate_values_names.size(); i++) {
            wheel.intermediateValuesMsg.names[i] = intermediate_values_names[i];
        }
    }
}

// Main update method with three-phase parallelism
void ApplyAllWheelsTerramechanicModel::OnUpdate()
{
    // Skip if not running
    if (plugin_state != PluginState::RUNNING) {
        return;
    }

    // // Rate limiter for performance
    // update_step_counter++;
    // if (options.skip_update_steps > 0) {
    //     if (update_step_counter % (options.skip_update_steps + 1) != 0) {
    //         // On skipped steps, just apply previous forces to maintain behavior
    //         for (int i = 0; i < num_wheels; i++) {
    //             if (fabs(wheels[i].wheel_state_params.omega * wheels[i].wheel_params.r_s) >= 0.02) {
    //                 applyForce(i);
    //             }
    //         }
    //         return;
    //     }
    // }

    // 1. First collect all wheel states (serial)
    for (int i = 0; i < num_wheels; i++) {
        // Set contact frame and wheel state
        wheels[i].contact_frame_rot = wheels[i].steer_link->WorldPose().Rot();
        if (wheels[i].name == "Archimede_br_wheel_link" || wheels[i].name == "Archimede_fr_wheel_link") {
            wheels[i].contact_frame_rot = wheels[i].contact_frame_rot * ignition::math::Quaterniond(0, 0, M_PI);
        }
        wheels[i].contact_frame_rot.Normalize();

        // Set soil parameters based on terrain
        setSoilParams(i);

        // Set wheel state parameters
        setWheelStateParams(i);
    }

    // 2. Perform computations (parallel - safe)
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < num_wheels; i++) {
        // Skip static wheels
        if (fabs(wheels[i].wheel_state_params.omega * wheels[i].wheel_params.r_s) < 0.02) {
            continue;
        }

        // Computation-heavy tasks
        setTunedParams(i);
        computeWheelLoad(i);

        // Find sinkage and compute forces (computationally intensive)
        if (findSinkage(i)) {
            computeForces(i);
        }
    }

    // 3. Apply forces (serial - safe)
    for (int i = 0; i < num_wheels; i++) {
        if (fabs(wheels[i].wheel_state_params.omega * wheels[i].wheel_params.r_s) < 0.02) {
            wheels[i].wheel_link->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
            wheels[i].wheel_link->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
            continue;
        }

        // Interact with physics engine (not thread-safe)
        applyForce(i);
        
        // Publish results
        if (this->publish_results) {
            publishResults(i);
        }
    }
}



void ApplyAllWheelsTerramechanicModel::OnUpdateCompact()
{
    // Skip if not running
    if (plugin_state != PluginState::RUNNING) {
        return;
    }
    
    // 1. First collect all wheel states (serial)
    for (int i = 0; i < num_wheels; i++) {
        // set wheel frame as contact frame, always apply force (if wheel is moving)
        wheels[i].contact_frame_rot = wheels[i].steer_link->WorldPose().Rot();
        // rotate frames xy plane such that x axis points forward
        if (wheels[i].name == "Archimede_br_wheel_link" || wheels[i].name == "Archimede_fr_wheel_link") {
            wheels[i].contact_frame_rot = wheels[i].contact_frame_rot * ignition::math::Quaterniond(0, 0, M_PI);
        }
        wheels[i].contact_frame_rot.Normalize();
        
        wheels[i].wheel_params.r_s = wheels[i].wheel_params.r + 0.5 * wheels[i].wheel_params.h_g;
        
        // get wheel state, set wheel_states_params and compute slips
        setWheelStateParams(i);
        
        // manually set soil params
        // wheels[i].soil_params.name = "Sand_marsSim"; // [Soil_Direct_#90_sand, Sand_marsSim]
        setSoilParams(i);
    }
    
    // 2. Perform calculations (parallel - computationally intensive)
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < num_wheels; i++) {
        WheelData& wheel = wheels[i];
        
        // TODO: case wheel is static, decide what to do & condition (if needed, omega=0 breaks things for now)
        if (fabs(wheel.wheel_state_params.omega * wheel.wheel_params.r_s) < 0.02)
        {
            wheel.wheel_link->SetLinearVel(ignition::math::Vector3d(0,0,0));
            wheel.wheel_link->SetAngularVel(ignition::math::Vector3d(0,0,0));
            if(this->debug){ROS_INFO_STREAM("DB [" << wheel.name << "]: wheel is static");}
            continue;
        }
        
        if(this->debug){ROS_INFO_STREAM("DB [" << wheel.name << "]: wheels rel xy vel: " << wheel.wheel_state_params.v_x << ", " << wheel.wheel_state_params.v_y
                                                            << "\n\t\t\t\t\t\tomega: " << wheel.wheel_state_params.omega << ", slip: " << wheel.wheel_state_params.s << ", beta: " << 180/M_PI*wheel.wheel_state_params.beta);}
        
        setTunedParams(i);
        
        // compute wheel load
        computeWheelLoad(i);
        
        // find sinkage iteratively equalizing computed F_z with wheel load
        double h_min=0, h_max=1.5*wheel.wheel_params.r_s, max_err=wheel.forces.W*pow(10,-3), F_z=0;
        double A, B, sigma_m, tau_xm, tau_ym;
        // binary search algorithm
        while (fabs(F_z - wheel.forces.W) > fabs(max_err))
        {
            double h_0 = (h_max+h_min)/2;

            // simplified geometry
            double theta_f = acos(1 - h_0 / wheel.wheel_params.r);
            double theta_r = 0;
            double theta_m = theta_f / 2;

            // normal stress
            sigma_m = wheel.soil_params.k * pow(wheel.wheel_params.r * (cos(theta_m) - cos(theta_f)), wheel.soil_params.n);

            // shear stress
            double j_xm = wheel.wheel_params.r * (theta_f - theta_m - (1 - wheel.wheel_state_params.s) * (sin(theta_f) - sin(theta_m)));
            double j_ym = wheel.wheel_params.r * (1 - wheel.wheel_state_params.s) * (theta_f - theta_m) * tan(fabs(wheel.wheel_state_params.beta));
            tau_xm = (wheel.soil_params.c + sigma_m * tan(wheel.soil_params.phi)) * (1 - exp(-j_xm / wheel.soil_params.K));
            tau_ym = (wheel.soil_params.c + sigma_m * tan(wheel.soil_params.phi)) * (1 - exp(-j_ym / wheel.soil_params.K));

            // Force coefficients
            A = (cos(theta_m) - cos(theta_r)) / (theta_m - theta_r) + (cos(theta_m) - cos(theta_f)) / (theta_f - theta_m);
            B = (sin(theta_m) - sin(theta_r)) / (theta_m - theta_r) + (sin(theta_m) - sin(theta_f)) / (theta_f - theta_m);

            // Normal force
            F_z = wheel.wheel_params.b * (wheel.wheel_params.r * sigma_m * A + wheel.wheel_params.r_s * tau_xm * B);

            if (F_z < wheel.forces.W)
            {h_min = h_0;}
            else
            {h_max = h_0;}
            if ((h_max-h_min) < pow(10,-6))
            {
                ROS_ERROR_STREAM("Sinkage not found for wheel [" << wheel.name << "]: expected Fz = " << wheel.forces.W << ", computed Fz = " << F_z << ", h_min_max = [" << h_min << " - " << h_max << "]");
                break;
            }
            wheel.terramechanics_params.h_0 = h_0;
            wheel.terramechanics_params.theta_f = theta_f;
            wheel.terramechanics_params.theta_r = theta_r;
            wheel.terramechanics_params.theta_m = theta_m;
        }
        if(this->debug){ROS_INFO_STREAM("DB [" << wheel.name << "]: Sinkage found: " << wheel.terramechanics_params.h_0);}

        double C = (wheel.terramechanics_params.theta_f - wheel.terramechanics_params.theta_r) / 2;

        // compute forces/moments
        wheel.forces.F_x = wheel.wheel_params.b * (wheel.wheel_params.r_s * tau_xm * A - wheel.wheel_params.r * sigma_m * B);
        wheel.forces.F_y = std::copysign(wheel.wheel_params.b * wheel.wheel_params.r_s * tau_ym * C, - wheel.wheel_state_params.beta); // bulldozing neglected
        wheel.forces.F_z = F_z;
        wheel.forces.M_x = wheel.forces.F_y * wheel.wheel_params.r_s;
        wheel.forces.M_y = - wheel.wheel_params.b * pow(wheel.wheel_params.r_s, 2) * tau_xm * C;
        wheel.forces.M_z = wheel.forces.F_y * wheel.wheel_params.r_s * sin(wheel.terramechanics_params.theta_m);
    }
    
    // 3. Apply forces and publish results (serial)
    for (int i = 0; i < num_wheels; i++) {
        // TODO: case wheel is static, decide what to do & condition (if needed, omega=0 breaks things for now)
        if (fabs(wheels[i].wheel_state_params.omega * wheels[i].wheel_params.r_s) < 0.02)
        {
            wheels[i].wheel_link->SetLinearVel(ignition::math::Vector3d(0,0,0));
            wheels[i].wheel_link->SetAngularVel(ignition::math::Vector3d(0,0,0));
            if(this->debug){ROS_INFO_STREAM("DB [" << wheels[i].name << "]: wheel is static");}
            continue;
        }
        
        // apply forces/moments to the wheel
        applyForce(i);
        
        // publish computed forces/moments (all of them)
        if (this->publish_results)
        {publishResults(i);}
    }
}



void ApplyAllWheelsTerramechanicModel::applyForce(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];
    
  // case forces are in contact frame
  // contact frame: force -> z=0(keep gazebo's), xy = model's
    ignition::math::Vector3d force_to_add_contact(wheel.forces.F_x, wheel.forces.F_y, 0);
      // M_y = M_z = 0, they are drive and steer joints resistent moments, but do not influence the rover movement
    ignition::math::Vector3d torque_to_add_contact(wheel.forces.M_x, 0, 0);
      // ignition::math::Vector3d torque_to_add_contact(this->forces.M_x, this->forces.M_y, this->forces.M_z);

    // Transform to world frame
    ignition::math::Vector3d force_to_add = wheel.contact_frame_rot.RotateVector(force_to_add_contact);
    ignition::math::Vector3d torque_to_add = wheel.contact_frame_rot.RotateVector(torque_to_add_contact);
    
  // apply force/torque in world frame to the wheel
    wheel.wheel_link->AddForce(force_to_add);// force in world frame
    wheel.wheel_link->AddTorque(torque_to_add);// torque in world frame
    
    if (this->debug) {
        ROS_INFO_STREAM("DB [" << wheel.name << "]: applied forces: " << 
                        force_to_add.X() << " - " << force_to_add.Y() << " - " << 
                        force_to_add.Z() << "\n\t\t\t\t\t\t\t\t\t\ttorques: " << 
                        torque_to_add.X() << " - " << torque_to_add.Y() << " - " << 
                        torque_to_add.Z());
    }
}


void ApplyAllWheelsTerramechanicModel::setWheelParams(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];
    
    // Set wheel params
    wheel.wheel_params.type = "grousers"; // smooth/grousers
    wheel.wheel_params.r = 0.085;
    wheel.wheel_params.b = 0.09;
    
    if (wheel.wheel_params.type == "smooth") {
        wheel.wheel_params.h_g = 0;
        wheel.wheel_params.mu = 1;
    } else {
        if (wheel.wheel_params.type != "grousers") {
            ROS_WARN("Unrecognized wheel type, defaulting to [grousers]...");
            wheel.wheel_params.type = "grousers";
        }
        wheel.wheel_params.h_g = 0.01;
        wheel.wheel_params.mu = 0.5;
    }
    
    wheel.wheel_params.r_s = wheel.wheel_params.r + wheel.wheel_params.h_g;
}

void ApplyAllWheelsTerramechanicModel::setSoilParams(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];
    
    // Get terrain below wheel
    std::string terrain_below_name;
    double terrain_below_distance; // unused
    wheel.wheel_link->GetNearestEntityBelow(terrain_below_distance, terrain_below_name);
    
    // TODO: the mapping could be done in a separate file (.yaml or something) instead of here, it depends on world models
    // map model-soil type for khalid's world
    if (terrain_below_name.find("terrain_blue") != std::string::npos) {
        wheel.soil_params.name = "Soil_Direct_#90_sand";
    } else if (terrain_below_name.find("terrain_red") != std::string::npos) {
        wheel.soil_params.name = "Sand_marsSim";
    } else if (terrain_below_name.find("terrain_green") != std::string::npos) {
        wheel.soil_params.name = "Clay";
    } else if (terrain_below_name.find("terrain_yellow") != std::string::npos) {
        wheel.soil_params.name = "Dry_sand";
    } else {
        // Default soil type
        wheel.soil_params.name = "Sand_marsSim";
    }
    
    
    if (wheel.soil_params.name == "Soil_Direct_#90_sand") {
        // from Pavlov (2024)
        wheel.soil_params.k = 8000 * pow(10, 3);
        wheel.soil_params.phi = 29 * M_PI/180;
        wheel.soil_params.rho = 13.03 * pow(10, 3);
        wheel.soil_params.c = 1.0 * pow(10, 3);
        wheel.soil_params.K = 0.021;
        // n0, n1, n2 are assigned later in setTunedParams
    } else if (wheel.soil_params.name == "Clay") {
        // from Ding et al. (2015), table I
        wheel.soil_params.k_c = 13.19 * pow(10, 3);
        wheel.soil_params.k_phi = 692.15 * pow(10, 3);
        wheel.soil_params.k = (wheel.soil_params.k_c / wheel.wheel_params.b + wheel.soil_params.k_phi);
        wheel.soil_params.phi = 13 * M_PI/180;
        wheel.soil_params.c = 4.14 * pow(10, 3);
        wheel.soil_params.K = 6 * pow(10, -3);
        wheel.soil_params.n = 0.5;
        
        wheel.tuned_params.n0 = NAN;
        wheel.tuned_params.n1 = NAN;
        wheel.tuned_params.n2 = NAN;
    } else if (wheel.soil_params.name == "Dry_sand") {
        // from Ding et al. (2015), table I
        wheel.soil_params.k_c = 0.99 * pow(10, 3);
        wheel.soil_params.k_phi = 1528.43 * pow(10, 3);
        wheel.soil_params.k = (wheel.soil_params.k_c / wheel.wheel_params.b + wheel.soil_params.k_phi);
        wheel.soil_params.phi = 28 * M_PI/180;
        wheel.soil_params.c = 1.04 * pow(10, 3);
        wheel.soil_params.K = 10 * pow(10, -3); // [10-25]*1e3, 10: firm sand - 25: loose sand
        wheel.soil_params.n = 1.10;
        
        wheel.tuned_params.n0 = NAN;
        wheel.tuned_params.n1 = NAN;
        wheel.tuned_params.n2 = NAN;
    } 


    else if (wheel.soil_params.name == "Lunar_soil")
    { // from Ding et al. (2015), table I
        wheel.soil_params.k_c = 1.4 * pow(10,3);
        wheel.soil_params.k_phi = 820 * pow(10,3);
        wheel.soil_params.k = (wheel.soil_params.k_c / wheel.wheel_params.b + wheel.soil_params.k_phi);
        wheel.soil_params.phi = 35 * M_PI/180; // [25-50]*pi/180
        wheel.soil_params.c = 0.17 * pow(10,3); // [0.1-2.7]*1e3
        wheel.soil_params.K = 18 * pow(10,-3);

        wheel.soil_params.n = 1.0;

        wheel.tuned_params.n0 = NAN;
        wheel.tuned_params.n1 = NAN;
        wheel.tuned_params.n2 = NAN;
    }
    else if (wheel.soil_params.name == "HIT-LSS1")
    { // from Ding et al. (2015), table I
        wheel.soil_params.k_c = 15.6 * pow(10,3);
        wheel.soil_params.k_phi = 2407.4 * pow(10,3);
        wheel.soil_params.k = (wheel.soil_params.k_c / wheel.wheel_params.b + wheel.soil_params.k_phi);
        wheel.soil_params.phi = 31.9 * M_PI/180;
        wheel.soil_params.c = 0.25 * pow(10,3);
        wheel.soil_params.K = 10 * pow(10,-3); // [9.7-13.1]*1e-3

        wheel.soil_params.n = 1.10;

        wheel.tuned_params.n0 = NAN;
        wheel.tuned_params.n1 = NAN;
        wheel.tuned_params.n2 = NAN;
    }


    else {
        // Default to Sand_marsSim
        if (wheel.soil_params.name != "Sand_marsSim") {
            ROS_WARN("Unrecognized soil name! Defaulting to [Sand_marsSim]...");
            wheel.soil_params.name = "Sand_marsSim";
        }
        
        // from Zhou et al. 2023
        wheel.soil_params.k_c = 13.6 * pow(10, 3);
        wheel.soil_params.k_phi = 2259.1 * pow(10, 3);
        wheel.soil_params.k = (wheel.soil_params.k_c / wheel.wheel_params.b + wheel.soil_params.k_phi);
        wheel.soil_params.c = 0.4623 * pow(10, 3);
        wheel.soil_params.phi = 35 * M_PI/180;
        wheel.soil_params.K = 0.015;
        
        wheel.tuned_params.n0 = 0.92;
        wheel.tuned_params.n1 = 0.5;
        wheel.tuned_params.n2 = wheel.tuned_params.n1;
    }
    
    wheel.soil_params.X_c = 45 * M_PI/180 - wheel.soil_params.phi/2;
}









void ApplyAllWheelsTerramechanicModel::setWheelStateParams(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];
    
    // Wheel angular drive velocity
    wheel.wheel_state_params.omega = wheel.wheel_link->RelativeAngularVel().Y();
    
    // Change sign where necessary so that omega > 0 for forward movement
    if (wheel.name == "Archimede_br_wheel_link" || wheel.name == "Archimede_fr_wheel_link") {
        wheel.wheel_state_params.omega *= -1;
    }
    
    // If omega < 0 (backward movement), change its sign and rotate contact frame so it's the same as moving forward in the reverse direction
    if (wheel.wheel_state_params.omega < 0) {
        wheel.wheel_state_params.omega *= -1;
        wheel.contact_frame_rot = wheel.contact_frame_rot * ignition::math::Quaterniond(0, 0, M_PI);
        wheel.contact_frame_rot.Normalize();
    }
    
    // Wheel velocity in contact frame
    ignition::math::Vector3d link_relative_vel = 
        wheel.contact_frame_rot.RotateVectorReverse(wheel.steer_link->WorldLinearVel());
        
    wheel.wheel_state_params.v_x = link_relative_vel.X();
    wheel.wheel_state_params.v_y = link_relative_vel.Y();
    
    wheel.wheel_state_params.v = sqrt(pow(wheel.wheel_state_params.v_x, 2) + 
                                     pow(wheel.wheel_state_params.v_y, 2));
                                     
    wheel.wheel_state_params.beta = atan(wheel.wheel_state_params.v_y / wheel.wheel_state_params.v_x);
    
    if (fabs(wheel.wheel_state_params.omega * wheel.wheel_params.r_s) <= pow(10, -4) && 
        fabs(wheel.wheel_state_params.v_x) <= pow(10, -4)) {
        // Case 0/0
        wheel.wheel_state_params.s = 0;
    } else if (fabs(wheel.wheel_params.r_s * wheel.wheel_state_params.omega) >= 
               fabs(wheel.wheel_state_params.v_x)) {
        // Slip
        wheel.wheel_state_params.s = 
            (wheel.wheel_params.r_s * wheel.wheel_state_params.omega - wheel.wheel_state_params.v_x) / 
            (wheel.wheel_params.r_s * wheel.wheel_state_params.omega);
    } else {
        // Skid
        wheel.wheel_state_params.s = 
            (wheel.wheel_params.r_s * wheel.wheel_state_params.omega - wheel.wheel_state_params.v_x) / 
            (wheel.wheel_state_params.v_x);
    }
}





void ApplyAllWheelsTerramechanicModel::setTunedParams(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];
    double beta = fabs(wheel.wheel_state_params.beta);
    
    if (wheel.wheel_params.type == "smooth" && wheel.soil_params.name == "Soil_Direct_#90_sand") {
    // coeff interpolated from Pavlov data (wheels won't be the same), not used in simplified geometry
        wheel.tuned_params.a0 = -0.0539*pow(beta, 3) - 0.0227*pow(beta, 2) + 0.6294*beta + 0.1674;
        wheel.tuned_params.a1 = 0.0156*pow(beta, 3) + 0.3414*pow(beta, 2) - 1.0039*beta + 0.7632;
        wheel.tuned_params.b0 = -0.4386*pow(beta, 4) + 0.8719*pow(beta, 3) - 0.5565*pow(beta, 2) + 0.0905*beta - 0.4753;
        wheel.tuned_params.b1 = 0.4178*pow(beta, 3) - 0.8113*pow(beta, 2) + 0.2433*beta - 0.0044;
        
        wheel.tuned_params.n0 = 1.46;
        wheel.tuned_params.n1 = 0.01;
        wheel.tuned_params.n2 = 0.55;
    } else if (wheel.wheel_params.type == "grousers" && wheel.soil_params.name == "Soil_Direct_#90_sand") {
        wheel.tuned_params.a0 = 0.7450*pow(beta, 4) - 2.4800*pow(beta, 3) + 2.6033*pow(beta, 2) - 0.4332*beta + 0.2716;
        wheel.tuned_params.a1 = -0.6876*pow(beta, 4) + 2.3687*pow(beta, 3) - 2.5096*pow(beta, 2) + 0.3618*beta + 0.6761;
        wheel.tuned_params.b0 = -0.3198*pow(beta, 3) + 0.1680*pow(beta, 2) + 0.3219*beta - 0.6372;
        wheel.tuned_params.b1 = 0.1387*pow(beta, 3) + 0.3971*pow(beta, 2) - 0.8644*beta - 0.1921;
        
        wheel.tuned_params.n0 = 1.46;
        wheel.tuned_params.n1 = 0.01;
        wheel.tuned_params.n2 = 0.74;
    }
    
    wheel.tuned_params.d0 = 1;
    wheel.tuned_params.d1 = 0.5;
    
      // case n is already set for the soil
    if (std::isnan(wheel.tuned_params.n0) || std::isnan(wheel.tuned_params.n1) || 
        std::isnan(wheel.tuned_params.n2)) {
        return;
    }
    
    if (wheel.wheel_state_params.s >= 0) {
        wheel.soil_params.n = wheel.tuned_params.n0 + wheel.tuned_params.n1 * wheel.wheel_state_params.s;
    } else {
        wheel.soil_params.n = wheel.tuned_params.n0 - wheel.tuned_params.n2 * wheel.wheel_state_params.s;
    }
}


bool ApplyAllWheelsTerramechanicModel::findSinkage(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];
    
      // find sinkage iteratively equalizing computed F_z with wheel load
    double h_min = 0;
    double h_max = 1.5 * wheel.wheel_params.r_s;
    double max_err = wheel.forces.W * pow(10, -3);
    double F_z = 0;
    
    // binary search algorithm
    while (fabs(F_z - wheel.forces.W) > fabs(max_err)) {
        wheel.terramechanics_params.h_0 = (h_max + h_min) / 2;
        
        computeContactGeometry(wheel_idx);
        computeStresses(wheel_idx);
        computeForces(wheel_idx, &F_z);
        
        if (F_z < wheel.forces.W) {
            h_min = wheel.terramechanics_params.h_0;
        } else {
            h_max = wheel.terramechanics_params.h_0;
        }
        
        if ((h_max - h_min) < pow(10, -6)) {
            ROS_ERROR_STREAM("Sinkage not found for wheel [" << wheel.name << 
                            "]: expected Fz = " << wheel.forces.W << 
                            ", computed Fz = " << F_z << 
                            ", h_min_max = [" << h_min << " - " << h_max << "]");
            return false;
        }
    }
    
    if (this->debug) {
        ROS_INFO_STREAM("DB [" << wheel.name << "]: Sinkage found: " << 
                        wheel.terramechanics_params.h_0);
    }
    
    return true;
}





void ApplyAllWheelsTerramechanicModel::computeContactGeometry(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];
    
    // unpack params
    double h_0 = wheel.terramechanics_params.h_0;
    double r_s = wheel.wheel_params.r_s;
    int rim_pts = this->options.rim_pts;
    double theta_f = acos(1 - h_0/r_s);

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
    double theta_r = 0;
    double theta_m = (theta_f + theta_r) / 2;
    
    double d_theta = (theta_f - theta_r) / (rim_pts - 1);
    for (int i = 0; i < rim_pts; i++) {
        wheel.terramechanics_params.theta[i] = theta_r + i * d_theta;
        wheel.terramechanics_params.h[i] = r_s * (cos(wheel.terramechanics_params.theta[i]) - cos(theta_f));
    }
    
      // update params
    wheel.terramechanics_params.theta_f = theta_f;
    wheel.terramechanics_params.theta_r = theta_r;
    wheel.terramechanics_params.theta_m = theta_m;
}



void ApplyAllWheelsTerramechanicModel::computeStresses(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];
  // unpack params
  int rim_pts = this->options.rim_pts;
  double k = wheel.soil_params.k;
  double c = wheel.soil_params.c;
  double phi = wheel.soil_params.phi;
  double K = wheel.soil_params.K;
  double n = wheel.soil_params.n;
  double r = wheel.wheel_params.r;
  double r_s = wheel.wheel_params.r_s;
  double mu = wheel.wheel_params.mu;
  double d0 = wheel.tuned_params.d0;
  double d1 = wheel.tuned_params.d1;
  double v_y = wheel.wheel_state_params.v_y;
  double omega = wheel.wheel_state_params.omega;
  double s = wheel.wheel_state_params.s;
  double theta_f = wheel.terramechanics_params.theta_f;
  double theta_r = wheel.terramechanics_params.theta_r;
  double theta_m = wheel.terramechanics_params.theta_m;

  double theta_0 = theta_f * (d0 + d1 * s);
  wheel.terramechanics_params.theta_0 = theta_0;

  std::ostringstream out_info; out_info << "DB [" << wheel.name << "]:";
  for (size_t i = 0; i < rim_pts; i++) {
      double theta_i = wheel.terramechanics_params.theta[i];
    // normal stress
    if (theta_i >= theta_m)
    {wheel.terramechanics_params.theta_e[i] = theta_i;}
    else {
      wheel.terramechanics_params.theta_e[i] = theta_f - 
          (theta_i - theta_r) * (theta_f - theta_m) / (theta_m - theta_r);
    }

    double sigma_r, sigma_rs;
    if (fabs(theta_i - theta_f) < pow(10,-6))
    {
      // if (cos(theta_e[i])-cos(theta_f)) is slightly negative (when we should be in theta_f), pow gives nan values
      sigma_r = 0;
      sigma_rs = 0;
    } else {
        sigma_r = k * pow(r, n) * 
            pow((cos(wheel.terramechanics_params.theta_e[i]) - cos(theta_f)), n);
        sigma_rs = k * pow(r_s, n) * 
            pow((cos(wheel.terramechanics_params.theta_e[i]) - cos(theta_f)), n);
      }

    // if(this->debug){out_info << "\n\ttheta_i = "<< theta[i]<<"\n\tsigma_rs: " << sigma_rs << " = " << k * pow(r_s,n) << " * pow(" << (cos(theta_e[i])-cos(theta_f))<<","<<n<<")";}

    wheel.terramechanics_params.sigma[i] = mu * sigma_rs + (1 - mu) * sigma_r;

    // shear stress
    wheel.terramechanics_params.v_jl[i] = v_y;
    wheel.terramechanics_params.j_l[i] = (theta_f - theta_i) * v_y / omega;
    // if(this->debug){out_info <<"\n\tv_jl = " << v_jl[i] << ", j_l = " << j_l[i];}

    if (s >= 0) {
        wheel.terramechanics_params.v_jt[i] = omega * r_s * (1 - (1 - s) * cos(theta_i));
        wheel.terramechanics_params.j_t[i] = r_s * ((theta_f - theta_i) - 
            (1 - s) * (sin(theta_f) - sin(theta_i)));
    } else if (theta_i >= theta_0) {
        wheel.terramechanics_params.v_jt[i] = omega * r_s / (1 + s) * 
            ((sin(theta_f) - sin(theta_0)) / (theta_f - theta_0) - cos(theta_i));
        wheel.terramechanics_params.j_t[i] = r_s / (1 + s) * 
            ((sin(theta_f) - sin(theta_0)) * (theta_f - theta_i) / (theta_f - theta_0) - 
              (sin(theta_f) - sin(theta_i)));
    } else {
        wheel.terramechanics_params.v_jt[i] = omega * r_s * (1 - cos(theta_i) / (1 + s));
        wheel.terramechanics_params.j_t[i] = r_s * ((theta_0 - theta_i) - 
            (sin(theta_0) - sin(theta_i)) / (1 + s));
    }

    wheel.terramechanics_params.j[i] = sqrt(pow(wheel.terramechanics_params.j_t[i], 2) + 
                                          pow(wheel.terramechanics_params.j_l[i], 2));

    wheel.terramechanics_params.tau[i] = (c + wheel.terramechanics_params.sigma[i] * tan(phi)) * 
                                        (1 - exp(-wheel.terramechanics_params.j[i] / K));
        double denominator = sqrt(pow(wheel.terramechanics_params.v_jt[i], 2) + 
                                  pow(wheel.terramechanics_params.v_jl[i], 2));
                                  
    wheel.terramechanics_params.tau_t[i] = wheel.terramechanics_params.tau[i] * 
                                          wheel.terramechanics_params.v_jt[i] / denominator;
                                          
    wheel.terramechanics_params.tau_l[i] = wheel.terramechanics_params.tau[i] * 
                                          wheel.terramechanics_params.v_jl[i] / denominator;

    // if(this->debug){out_info << "\n\tsigma = " << sigma[i] << ", tau = " << tau[i];}
  }
  // // if(this->debug){ROS_INFO_STREAM(out_info.str());}

  // // update params
  // std::memcpy(this -> terramechanics_params.theta_e, theta_e, rim_pts * sizeof(double));
  // std::memcpy(this -> terramechanics_params.sigma, sigma, rim_pts * sizeof(double));
  // std::memcpy(this -> terramechanics_params.tau, tau, rim_pts * sizeof(double));
  // std::memcpy(this -> terramechanics_params.tau_t, tau_t, rim_pts * sizeof(double));
  // std::memcpy(this -> terramechanics_params.tau_l, tau_l, rim_pts * sizeof(double));
  // std::memcpy(this -> terramechanics_params.j, j, rim_pts * sizeof(double));
  // std::memcpy(this -> terramechanics_params.j_t, j_t, rim_pts * sizeof(double));
  // std::memcpy(this -> terramechanics_params.j_l, j_l, rim_pts * sizeof(double));
  // std::memcpy(this -> terramechanics_params.v_jt, v_jt, rim_pts * sizeof(double));
  // std::memcpy(this -> terramechanics_params.v_jl, v_jl, rim_pts * sizeof(double));
  // this -> terramechanics_params.theta_0 = theta_0;
}



void ApplyAllWheelsTerramechanicModel::computeForces(int wheel_idx, double* F_z_out)
{
  // if F_z_out is passed as an argument this only computes F_z and assignes it to F_z_out,
  // otherwise it computes all forces/moments and updates the forces params
  WheelData& wheel = wheels[wheel_idx];
  // unpack params
  int rim_pts = this -> options.rim_pts;
  std::string bulldozing_resistence = this -> options.bulldozing_resistence;
  double X_c = wheel.soil_params.X_c;
  double c = wheel.soil_params.c;
  double rho = wheel.soil_params.rho;
  double phi = wheel.soil_params.phi;
  double theta_f = wheel.terramechanics_params.theta_f;
  double theta_r = wheel.terramechanics_params.theta_r;
  double theta_m = wheel.terramechanics_params.theta_m;
  double beta = wheel.wheel_state_params.beta;
  double r_s = wheel.wheel_params.r_s;
  double b = wheel.wheel_params.b;


  double d_theta = (theta_f-theta_r) / (rim_pts - 1);
  double F_z = 0;

  for (size_t i = 1; i < rim_pts; i++) {
      F_z += r_s * b * d_theta * (wheel.terramechanics_params.tau_t[i] * 
                                sin(wheel.terramechanics_params.theta[i]) + 
                                wheel.terramechanics_params.sigma[i] * 
                                cos(wheel.terramechanics_params.theta[i]));
  }
  if (F_z_out)
  {
    *F_z_out = F_z;
    return;
  }

  double F_x = 0, F_y = 0, M_x = 0, M_y = 0, M_z = 0;

  if (bulldozing_resistence == "Ishigami"){
    double D1 = 1/tan(X_c) + tan(X_c + phi);
    double D2 = 1/tan(X_c) + pow((1/tan(X_c)), 2) / (1/tan(phi));
        
    for (size_t i = 1; i < rim_pts; i++) {
        double hh = r_s * (cos(wheel.terramechanics_params.theta[i]) - cos(theta_f));
        wheel.forces.R_b[i] = D1 * (c * hh + 0.5 * D2 * rho * pow(hh, 2));
        
        // negative sign so that F_y has opposite direction to wheel lateral velocity
        F_y -= d_theta * (r_s * b * wheel.terramechanics_params.tau_l[i] + 
                        wheel.forces.R_b[i] * sin(beta) * 
                        (r_s - hh * cos(wheel.terramechanics_params.theta[i])));
        
        F_x += r_s * b * d_theta * (wheel.terramechanics_params.tau_t[i] * 
                                  cos(wheel.terramechanics_params.theta[i]) - 
                                  wheel.terramechanics_params.sigma[i] * 
                                  sin(wheel.terramechanics_params.theta[i]));
                    M_y -= pow(r_s, 2) * b * d_theta * wheel.terramechanics_params.tau_t[i];
    }   
 
  } else if (bulldozing_resistence == "Pavlov"){
      for (size_t i = 1; i < rim_pts; i++) {
          double h_b = r_s * ((sin(wheel.terramechanics_params.theta[i]) - sin(theta_r)) * 
                            (cos(theta_r) - cos(theta_f)) / (sin(theta_f) - sin(theta_r)) - 
                            (cos(theta_r) - cos(wheel.terramechanics_params.theta[i])));
                            
          wheel.forces.R_b[i] = rho/2 * pow(h_b, 2) * pow((1/tan(X_c)), 2) * 
                              (1 + 0.5 / tan(X_c) * tan(phi)) + 
                              2 * h_b * c / tan(X_c);

      // negative sign so that F_y has opposite direction to wheel lateral velocity
      F_y -= r_s * d_theta * (b * wheel.terramechanics_params.tau_l[i] + 
                              wheel.forces.R_b[i] * sin(beta) * 
                              cos(wheel.terramechanics_params.theta[i]));
      
      F_x += r_s * b * d_theta * (wheel.terramechanics_params.tau_t[i] * 
                                cos(wheel.terramechanics_params.theta[i]) - 
                                wheel.terramechanics_params.sigma[i] * 
                                sin(wheel.terramechanics_params.theta[i]));
      
      M_y -= pow(r_s, 2) * b * d_theta * wheel.terramechanics_params.tau_t[i];
    }
  } else{
    if (bulldozing_resistence != "neglect"){
      ROS_WARN("bulldozing_resistence option not recognised. Neglecting sidewall bulldozing force...");
    }
    for (size_t i = 1; i < rim_pts; i++)
    {
      // negative sign so that F_y has opposite direction to wheel lateral velocity
      F_y -= d_theta * (r_s * b * wheel.terramechanics_params.tau_l[i]);

      F_x += r_s * b * d_theta * (wheel.terramechanics_params.tau_t[i] * 
                                cos(wheel.terramechanics_params.theta[i]) - 
                                wheel.terramechanics_params.sigma[i] * 
                                sin(wheel.terramechanics_params.theta[i]));

      M_y -= pow(r_s,2) * b * d_theta * wheel.terramechanics_params.tau_t[i];
    }
  }

  M_x += F_y * r_s;
  M_z += F_y * r_s * sin(theta_m);

  // update params
  wheel.forces.F_x = F_x;
  wheel.forces.F_y = F_y;
  wheel.forces.F_z = F_z;
  wheel.forces.M_x = M_x;
  wheel.forces.M_y = M_y;
  wheel.forces.M_z = M_z;
}





void ApplyAllWheelsTerramechanicModel::computeWheelLoad(int wheel_idx)
{
    WheelData& wheel = wheels[wheel_idx];
  // compute static load on the wheel and gravity component tangential to the ground according
  // to the rover orientation wrt to gravity direction
  // this does not account for dynamics effects (curves and acceleration/deceleration),
  // and always considers all 4 wheels in contact with the terrain

  ignition::math::Quaterniond rover_orient = this->model->WorldPose().Rot();

  // rover (and assumed terrain) inclinations around lateral & longitudinal axis
  double theta = -rover_orient.Pitch(); // positive -> forward is uphill
  double alpha = rover_orient.Roll(); // positive -> left is uphill

  double wheel_load = wheel.link_mass;

  for (std::vector<double> link_i : this->other_masses)
  {
    wheel_load += link_i[0] * (1 - (link_i[1] + wheel.f_signs[0]*link_i[3]*tan(theta))/this->rover_dimensions[0]) * (1 - (link_i[2] + wheel.f_signs[1]*link_i[3]*tan(alpha))/this->rover_dimensions[1]);
  }

  // wheel load along gravity direction (world negative z-axis)
  wheel_load *= this -> world_gravity;

  // load components in contact frame
  ignition::math::Vector3d wheel_load_contact = wheel.contact_frame_rot.RotateVectorReverse(ignition::math::Vector3d(0,0,-wheel_load));

  if (this->debug){ROS_INFO_STREAM("DB [" << wheel.name << "]: Load on wheel xyz:" << wheel_load_contact.X() << ", " << wheel_load_contact.Y() << ", " << wheel_load_contact.Z());}

  wheel.forces.W = fabs(wheel_load_contact.Z());
}






void ApplyAllWheelsTerramechanicModel::publishResults(int wheel_idx)
{
  WheelData& wheel = wheels[wheel_idx];

  // stamps  
  wheel.resultMsg.header.stamp = ros::Time::now();
  wheel.intermediateValuesMsg.header.stamp = ros::Time::now();

  // contact frame
  ignition::math::Vector3d force_contact(wheel.forces.F_x, wheel.forces.F_y, wheel.forces.F_z);
  ignition::math::Vector3d torque_contact(wheel.forces.M_x, wheel.forces.M_y, wheel.forces.M_z);

  // world frame
  ignition::math::Vector3d force_world = wheel.contact_frame_rot.RotateVector(force_contact);
  ignition::math::Vector3d torque_world = wheel.contact_frame_rot.RotateVector(torque_contact);

  wheel.resultMsg.vectors[0].x = force_contact.X();
  wheel.resultMsg.vectors[0].y = force_contact.Y();
  wheel.resultMsg.vectors[0].z = force_contact.Z();
  wheel.resultMsg.vectors[1].x = torque_contact.X();
  wheel.resultMsg.vectors[1].y = torque_contact.Y();
  wheel.resultMsg.vectors[1].z = torque_contact.Z();
  wheel.resultMsg.vectors[2].x = force_world.X();
  wheel.resultMsg.vectors[2].y = force_world.Y();
  wheel.resultMsg.vectors[2].z = force_world.Z();
  wheel.resultMsg.vectors[3].x = torque_world.X();
  wheel.resultMsg.vectors[3].y = torque_world.Y();
  wheel.resultMsg.vectors[3].z = torque_world.Z();

  // fill intermediate values msg
  wheel.intermediateValuesMsg.data[0] = wheel.wheel_state_params.omega;
  wheel.intermediateValuesMsg.data[1] = wheel.wheel_state_params.v_x;
  wheel.intermediateValuesMsg.data[2] = wheel.wheel_state_params.v_y;
  wheel.intermediateValuesMsg.data[3] = wheel.wheel_state_params.s;
  wheel.intermediateValuesMsg.data[4] = wheel.wheel_state_params.beta * 180/M_PI;
  wheel.intermediateValuesMsg.data[5] = wheel.forces.W;
  wheel.intermediateValuesMsg.data[6] = wheel.terramechanics_params.h_0;

  // publish
  wheel.ros_pub_results.publish(wheel.resultMsg);
  wheel.ros_pub_intermediate_values.publish(wheel.intermediateValuesMsg);
}