#include "orbbec_camera_lifecycle/lifecycle_camera_node.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <exception>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <utility>

namespace orbbec_camera_lifecycle {

LifecycleCameraNode::LifecycleCameraNode(
    const rclcpp::NodeOptions& options,
    const std::shared_ptr<rclcpp::Executor>& executor)
    : LifecycleNode("orbbec_camera_manager", options), executor_(executor) {
  config_file_ = declare_parameter<std::string>("config_file", "");
}

LifecycleCameraNode::~LifecycleCameraNode() { stop_drivers(); }

LifecycleCameraNode::CallbackReturn LifecycleCameraNode::on_configure(
    const rclcpp_lifecycle::State&) {
  if (!load_configuration()) {
    return CallbackReturn::FAILURE;
  }
  return CallbackReturn::SUCCESS;
}

LifecycleCameraNode::CallbackReturn LifecycleCameraNode::on_activate(
    const rclcpp_lifecycle::State&) {
  return start_drivers() ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

LifecycleCameraNode::CallbackReturn LifecycleCameraNode::on_deactivate(
    const rclcpp_lifecycle::State&) {
  stop_drivers();
  return CallbackReturn::SUCCESS;
}

LifecycleCameraNode::CallbackReturn LifecycleCameraNode::on_cleanup(
    const rclcpp_lifecycle::State&) {
  stop_drivers();
  return CallbackReturn::SUCCESS;
}

LifecycleCameraNode::CallbackReturn LifecycleCameraNode::on_shutdown(
    const rclcpp_lifecycle::State&) {
  stop_drivers();
  return CallbackReturn::SUCCESS;
}

LifecycleCameraNode::CallbackReturn LifecycleCameraNode::on_error(
    const rclcpp_lifecycle::State&) {
  stop_drivers();
  return CallbackReturn::SUCCESS;
}

bool LifecycleCameraNode::load_configuration() {
  cameras_.clear();
  if (config_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "config_file is required");
    return false;
  }
  try {
    // 相机列表
    const auto root = YAML::LoadFile(config_file_);
    const auto list = root["cameras"];
    if (!list || !list.IsSequence() || list.size() == 0) {
      RCLCPP_ERROR(get_logger(),
                   "config must contain a non-empty cameras sequence");
      return false;
    }

    // 逐项生成相机配置(只是参数与配置，没有实际节点创建)
    for (const auto& entry : list) {
      CameraInstance camera;
      camera.name = entry["name"].as<std::string>();
      // OBCameraNodeDriver创建的话题没有节点名前缀，加上namespace让话题有节点名前缀，避免话题冲突
      // 导致节点变为为/camera_name/camera_name这样的两层形式，话题名为/camera_name/color/image_raw
      camera.namespace_name = "/" + camera.name;
      if (camera.name.empty()) {
        throw std::runtime_error("camera name cannot be empty");
      }

      auto parameters = entry["parameters"];
      std::vector<rclcpp::Parameter> overrides;
      if (parameters && parameters.IsMap()) {
        for (const auto& item : parameters) {
          const auto key = item.first.as<std::string>();
          const auto value = item.second;
          if (!value.IsScalar()) {
            RCLCPP_WARN(get_logger(), "ignoring non-scalar parameter '%s'",
                        key.c_str());
            continue;
          }

          const auto text = value.as<std::string>();
          if (text == "true" || text == "false") {
            overrides.emplace_back(key, text == "true");
            continue;
          }

          try {
            overrides.emplace_back(key, value.as<int64_t>());
            continue;
          } catch (const YAML::Exception&) {
            // Values that are not integers may still be floating point or text.
          }

          try {
            overrides.emplace_back(key, value.as<double>());
          } catch (const YAML::Exception&) {
            overrides.emplace_back(key, text);
          }
        }
      }
      // 节点的参数配置
      camera.options.parameter_overrides(overrides);
      camera.options.context(get_node_base_interface()->get_context());
      camera.options.use_global_arguments(false);
      camera.options.arguments({"--ros-args", "-r", "__node:=" + camera.name,
                                "-r", "__ns:=" + camera.namespace_name});

      cameras_.push_back(std::move(camera));
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "failed to load config: %s", error.what());
    return false;
  }
  return true;
}

bool LifecycleCameraNode::start_drivers() {
  if (!cameras_.empty() && cameras_.front().driver) {
    return true;
  }

  auto executor = executor_.lock();
  if (!executor) {
    RCLCPP_ERROR(get_logger(), "executor is no longer available");
    return false;
  }

  try {
    if (!component_loader_) {
      auto loader_options = rclcpp::NodeOptions().use_global_arguments(false);
      component_loader_ = std::make_shared<rclcpp_components::ComponentManager>(
          executor_, "orbbec_component_loader", loader_options);
    }
    if (!driver_factory_) {
      const auto resources =
          component_loader_->get_component_resources("orbbec_camera");
      const auto resource = std::find_if(
          resources.begin(), resources.end(), [](const auto& candidate) {
            return candidate.first == "orbbec_camera::OBCameraNodeDriver";
          });
      if (resource == resources.end()) {
        RCLCPP_ERROR(get_logger(), "Orbbec camera component is not registered");
        return false;
      }
      driver_factory_ = component_loader_->create_component_factory(*resource);
    }
    for (auto& camera : cameras_) {
      camera.driver = std::make_unique<rclcpp_components::NodeInstanceWrapper>(
          driver_factory_->create_node_instance(camera.options));
      executor->add_node(camera.driver->get_node_base_interface());
      RCLCPP_INFO(get_logger(), "activated camera %s/%s",
                  camera.namespace_name.c_str(), camera.name.c_str());
    }
    return true;
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "failed to activate Orbbec driver: %s",
                 error.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(),
                 "failed to activate Orbbec driver: unknown exception");
  }
  return false;
}

void LifecycleCameraNode::stop_drivers() {
  if (auto executor = executor_.lock()) {
    for (auto& camera : cameras_)
      if (camera.driver) {
        try {
          executor->remove_node(camera.driver->get_node_base_interface());
        } catch (const std::exception& error) {
          RCLCPP_WARN(get_logger(), "failed to remove camera: %s",
                      error.what());
        }
        camera.driver.reset();
      }
  }
}

}  // namespace orbbec_camera_lifecycle

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<orbbec_camera_lifecycle::LifecycleCameraNode>(
      rclcpp::NodeOptions(), executor);
  executor->add_node(node->get_node_base_interface());
  executor->spin();
  executor->remove_node(node->get_node_base_interface());
  node.reset();
  executor.reset();
  rclcpp::shutdown();
  return 0;
}
