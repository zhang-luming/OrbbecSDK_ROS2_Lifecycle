#pragma once

#include <memory>
#include <rclcpp/executor.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rclcpp_components/node_factory.hpp>
#include <rclcpp_components/node_instance_wrapper.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <vector>

namespace orbbec_camera_lifecycle {

class LifecycleCameraNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  LifecycleCameraNode(const rclcpp::NodeOptions& options,
                      const std::shared_ptr<rclcpp::Executor>& executor);
  ~LifecycleCameraNode() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

 private:
  struct CameraInstance {
    std::string name;
    std::string namespace_name;
    rclcpp::NodeOptions options;
    std::unique_ptr<rclcpp_components::NodeInstanceWrapper> driver;
  };
  bool load_configuration();
  bool start_drivers();
  void stop_drivers();

  std::weak_ptr<rclcpp::Executor> executor_;
  std::shared_ptr<rclcpp_components::ComponentManager> component_loader_;
  std::shared_ptr<rclcpp_components::NodeFactory> driver_factory_;
  std::string config_file_;
  std::vector<CameraInstance> cameras_;
};

}  // namespace orbbec_camera_lifecycle
