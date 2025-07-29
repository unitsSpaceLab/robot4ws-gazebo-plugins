#pragma once

#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>

#include <rclcpp/rclcpp.hpp>
#include <robot4ws_msgs/msg/dynamixel_parameters1.hpp>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <yaml-cpp/yaml.h>
#include <gz/math/PID.hh>

#include <map>
#include <mutex>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace my_namespace
{

/// Struct to hold per-joint configuration
struct JointConfig
{
  std::string joint_name;
  std::string msg_field;
  std::string control_type; // "position" or "velocity"
  double coeff{1.0};

  gz::math::PID pid;        // Used only if control_type == position

  gz::sim::Entity joint_entity{gz::sim::kNullEntity};

  // Index to the field descriptor
  size_t field_index = 0;
};

/// Gazebo System Plugin
class JointsControllerPluginDynamixel
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override;

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override;

private:
  /// Load YAML config
  void LoadConfig(const std::string &path, gz::sim::EntityComponentManager &ecm);

  /// ROS2 callback
  void RosCallback(const robot4ws_msgs::msg::DynamixelParameters1::SharedPtr msg);

  gz::sim::Model model_{gz::sim::kNullEntity};

  /// Joint name → config
  std::map<std::string, JointConfig> joint_configs_;

  // ROS2
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<robot4ws_msgs::msg::DynamixelParameters1>::SharedPtr sub_;
  std::thread ros_thread_;
  std::mutex mutex_;

  /// Last received message
  robot4ws_msgs::msg::DynamixelParameters1 last_msg_;

  std::string topic_name_;
};

}  // namespace my_namespace
