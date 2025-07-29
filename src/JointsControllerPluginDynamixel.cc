#include "JointsControllerPluginDynamixel.hh"

#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/common/Profiler.hh>

#include <yaml-cpp/yaml.h>

using namespace my_namespace;

void JointsControllerPluginDynamixel::Configure(const gz::sim::Entity &entity,
                                        const std::shared_ptr<const sdf::Element> &sdf,
                                        gz::sim::EntityComponentManager &ecm,
                                        gz::sim::EventManager &)
{
  model_ = gz::sim::Model(entity);

  // Get sdf params
  topic_name_ = sdf->Get<std::string>("topic", "/cmd_vel_motors").first;
  std::string config_file = sdf->Get<std::string>("config_file", 
    ament_index_cpp::get_package_share_directory("robot4ws_gazebo_plugins")+"config/joints_control_config.yaml").first;

  // Get config from yaml file
  LoadConfig(config_file, ecm);

  // get field indx in msg
  auto ts = rosidl_typesupport_cpp::get_message_type_support_handle<robot4ws_msgs::msg::DynamixelParameters1>();
  auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
  for (auto & [_, jc] : joint_configs_)
  {
    bool found = false;
    for (size_t i = 0; i < members->member_count_; ++i)
    {;
      if (members->members_[i].name_ == jc.msg_field)
      {
        jc.field_index = i;
        found = true;
        break;
      }
    }

    if (!found)
    {
      gzerr << "Field " << jc.msg_field << " not found in [DynamixelParameters1] message!" << std::endl;
      throw std::runtime_error("Field not found");
    }
  }

  // ROS2
  rclcpp::init(0, nullptr);
  ros_node_ = std::make_shared<rclcpp::Node>("gazebo_sim_controller");
  sub_ = ros_node_->create_subscription<robot4ws_msgs::msg::DynamixelParameters1>(
    topic_name_, 10,
    std::bind(&JointsControllerPluginDynamixel::RosCallback, this, std::placeholders::_1));

  ros_thread_ = std::thread([this]() {
    rclcpp::spin(ros_node_);
  });

  gzdbg << "[JointsControllerPluginDynamixel] Subscribed to [" << topic_name_ << "] topic" << std::endl;
}

void JointsControllerPluginDynamixel::LoadConfig(const std::string &path, gz::sim::EntityComponentManager &ecm)
{
  YAML::Node config = YAML::LoadFile(path);

  for (const auto &node : config["joints"])
  {
    JointConfig jc;
    jc.joint_name = node["name"].as<std::string>();
    jc.msg_field = node["msg_field"].as<std::string>();
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
      gzerr << "[JointsControllerPluginDynamixel] Joint " << jc.joint_name << " not found!" << std::endl;
      continue;
    }

    joint_configs_[jc.joint_name] = jc;

    gzdbg << "[JointsControllerPluginDynamixel] Loaded config for joint: ["
          << jc.joint_name << "] as [" << jc.control_type << "]" << std::endl;
  }
}

void JointsControllerPluginDynamixel::RosCallback(const robot4ws_msgs::msg::DynamixelParameters1::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_msg_ = *msg;
}

void JointsControllerPluginDynamixel::PreUpdate(const gz::sim::UpdateInfo &info,
                                        gz::sim::EntityComponentManager &ecm)
{
  if (info.paused)
    return;

  std::lock_guard<std::mutex> lock(mutex_);

  const double dt = std::chrono::duration<double>(info.dt).count();

  auto ts = rosidl_typesupport_cpp::get_message_type_support_handle<robot4ws_msgs::msg::DynamixelParameters1>();
  auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(ts->data);
  const uint8_t * base_ptr = reinterpret_cast<const uint8_t *>(&last_msg_);

  for (auto &[name, jc] : joint_configs_)
  {
    double cmd_value = 0.0;

    const auto &member = members->members_[jc.field_index];

    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE)
    {
      const double *cmd_ptr = reinterpret_cast<const double *>(base_ptr + member.offset_);
      cmd_value = (*cmd_ptr) * jc.coeff;
    }
    else
    {
      gzerr << "Field " << jc.msg_field << " is not a double!\n";
      continue;
    }

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
  my_namespace::JointsControllerPluginDynamixel,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(my_namespace::JointsControllerPluginDynamixel,
                    "my_namespace::JointsControllerPluginDynamixel")
