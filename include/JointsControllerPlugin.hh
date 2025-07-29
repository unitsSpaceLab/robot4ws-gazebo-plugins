#pragma once

#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>

#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>

#include <yaml-cpp/yaml.h>
#include <map>
#include <thread>
#include <mutex>

#include <gz/math/PID.hh>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace my_namespace
{

struct JointConfig
{
  std::string joint_name;
  int msg_index;            // Index in Actuators message
  std::string control_type; // "position" or "velocity"
  double coeff{1.0};
  gz::math::PID pid;        // Used only if control_type == position
  gz::sim::Entity joint_entity{gz::sim::kNullEntity};
};

class JointsControllerPlugin
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
  void ActuatorsCallback(const gz::msgs::Actuators &_msg);

  gz::sim::Model model_{gz::sim::kNullEntity};

  /// Joint name → config
  std::map<std::string, JointConfig> joint_configs_;

  gz::transport::Node gz_node_;
  std::string gz_topic_;
  std::mutex mutex_;
  gz::msgs::Actuators last_msg_;
};

}
