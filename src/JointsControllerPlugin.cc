#include "JointsControllerPlugin.hh"

#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

#include <yaml-cpp/yaml.h>

using namespace my_namespace;

void JointsControllerPlugin::Configure(const gz::sim::Entity &entity,
                                        const std::shared_ptr<const sdf::Element> &sdf,
                                        gz::sim::EntityComponentManager &ecm,
                                        gz::sim::EventManager &)
{
  model_ = gz::sim::Model(entity);

  gz_topic_ = sdf->Get<std::string>("topic", "/cmd_vel_motors").first;
  std::string config_file = sdf->Get<std::string>("config_path", 
    ament_index_cpp::get_package_share_directory("robot4ws_gazebo_plugins")+"/config/joints_control_config.yaml").first;

  LoadConfig(config_file, ecm);

  // Gazebo transport subscription
  if (!gz_node_.Subscribe(gz_topic_, &JointsControllerPlugin::ActuatorsCallback, this))
  {
    gzerr << "Failed to subscribe to [" << gz_topic_ << "]" << std::endl;
  }

  gzdbg << "JointsControllerPlugin subscribed to [" << gz_topic_ << "]" << std::endl;
}

void JointsControllerPlugin::LoadConfig(const std::string &path,
                                         gz::sim::EntityComponentManager &ecm)
{
  YAML::Node config = YAML::LoadFile(path);

  for (const auto &node : config["joints"])
  {
    JointConfig jc;
    jc.joint_name = node["name"].as<std::string>();
    jc.msg_index = node["msg_index"].as<int>();
    jc.control_type = node["control_type"].as<std::string>();
    jc.coeff = node["coeff"].as<double>();

    if (jc.control_type == "position")
    {
      double p = node["pid"]["p"].as<double>();
      double i = node["pid"]["i"].as<double>();
      double d = node["pid"]["d"].as<double>();
      jc.pid = gz::math::PID(p, i, d);
    }

    jc.joint_entity = model_.JointByName(ecm, jc.joint_name);
    if (jc.joint_entity == gz::sim::kNullEntity)
    {
      gzerr << "Joint " << jc.joint_name << " not found!" << std::endl;
      continue;
    }

    joint_configs_[jc.joint_name] = jc;

    gzdbg << "[JointsControllerPlugin] Loaded config for joint: ["
          << jc.joint_name << "] as [" << jc.control_type << "]" << std::endl;
  }
}

void JointsControllerPlugin::ActuatorsCallback(const gz::msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_msg_ = _msg;
}

void JointsControllerPlugin::PreUpdate(const gz::sim::UpdateInfo &info,
                                        gz::sim::EntityComponentManager &ecm)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (info.paused) return;

  const double dt = std::chrono::duration<double>(info.dt).count();

  for (auto &[name, jc] : joint_configs_)
  {
    double cmd_value = 0.0;

    if (jc.control_type == "velocity")
    {
      if (jc.msg_index < last_msg_.velocity_size())
        cmd_value = last_msg_.velocity(jc.msg_index);
    }
    else if (jc.control_type == "position")
    {
      if (jc.msg_index < last_msg_.position_size())
        cmd_value = last_msg_.position(jc.msg_index);
    }

    cmd_value *= jc.coeff;

    if (jc.control_type == "velocity")
    {
      ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(jc.joint_entity, {cmd_value});
    }
    else if (jc.control_type == "position")
    {
      auto posComp = ecm.Component<gz::sim::components::JointPosition>(jc.joint_entity);
      if (!posComp || posComp->Data().empty())
        continue;

      double current_pos = posComp->Data()[0];
      double error = current_pos - cmd_value;
      double vel_cmd = jc.pid.Update(error, std::chrono::duration<double>(dt));

      ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(jc.joint_entity, {vel_cmd});
    }
  }
}

// Register plugin
GZ_ADD_PLUGIN(
  my_namespace::JointsControllerPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(my_namespace::JointsControllerPlugin,
                    "my_namespace::JointsControllerPlugin")
