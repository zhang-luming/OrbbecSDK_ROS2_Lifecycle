#include "orbbec_camera_lifecycle/camera_self_check.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <exception>
#include <iomanip>
#include <sstream>
#include <time.h>
#include <utility>

namespace orbbec_camera_lifecycle {

namespace {

uint64_t monotonic_raw_nanoseconds() {
  timespec timestamp{};
  clock_gettime(CLOCK_MONOTONIC_RAW, &timestamp);
  return static_cast<uint64_t>(timestamp.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(timestamp.tv_nsec);
}

}  // namespace

CameraSelfCheck::CameraSelfCheck(const rclcpp::NodeOptions& options)
    : LifecycleNode("orbbec_self_test_node", options) {
  declare_parameter("config_file", "");
  declare_parameter("service_name", service_name_);
  declare_parameter("status_topic", status_topic_);
  declare_parameter("target_fps", target_fps_);
  declare_parameter("fps_tolerance", fps_tolerance_);
  declare_parameter("test_duration_sec", test_duration_sec_);
  declare_parameter("first_frame_timeout_sec", first_frame_timeout_sec_);
  declare_parameter("status_rate_hz", status_rate_hz_);
}

CameraSelfCheck::CallbackReturn CameraSelfCheck::on_configure(
    const rclcpp_lifecycle::State&) {
  config_file_ = get_parameter("config_file").as_string();
  if (config_file_.empty()) {
    config_file_ = ament_index_cpp::get_package_share_directory(
                       "orbbec_camera_lifecycle") +
                   "/config/cameras.yaml";
  }
  service_name_ = get_parameter("service_name").as_string();
  status_topic_ = get_parameter("status_topic").as_string();
  target_fps_ = get_parameter("target_fps").as_double();
  fps_tolerance_ = get_parameter("fps_tolerance").as_double();
  test_duration_sec_ = get_parameter("test_duration_sec").as_double();
  first_frame_timeout_sec_ =
      get_parameter("first_frame_timeout_sec").as_double();
  status_rate_hz_ = get_parameter("status_rate_hz").as_double();

  if (target_fps_ <= 0.0 || fps_tolerance_ < 0.0 ||
      test_duration_sec_ <= 0.0 || first_frame_timeout_sec_ <= 0.0 ||
      status_rate_hz_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "self-test timing and FPS parameters are invalid");
    return CallbackReturn::FAILURE;
  }

  streams_.clear();
  try {
    const auto root = YAML::LoadFile(config_file_);
    const auto cameras = root["cameras"];
    if (!cameras || !cameras.IsSequence()) {
      RCLCPP_ERROR(get_logger(), "config must contain a cameras sequence");
      return CallbackReturn::FAILURE;
    }

    for (const auto& camera : cameras) {
      if (camera["enabled"] && !camera["enabled"].as<bool>()) continue;
      const auto ns = camera["namespace"]
                          ? camera["namespace"].as<std::string>()
                          : camera["name"].as<std::string>();
      const auto parameters = camera["parameters"];
      const auto add_stream = [this, &ns](const char* name, bool enabled) {
        if (!enabled) return;
        Stream stream;
        stream.topic = "/" + ns + "/" + name + "/image_raw";
        streams_.push_back(std::move(stream));
      };
      add_stream("color", !parameters || !parameters["enable_color"] ||
                              parameters["enable_color"].as<bool>());
      add_stream("depth", !parameters || !parameters["enable_depth"] ||
                              parameters["enable_depth"].as<bool>());
      add_stream("left_ir", parameters && parameters["enable_left_ir"] &&
                                parameters["enable_left_ir"].as<bool>());
      add_stream("right_ir", parameters && parameters["enable_right_ir"] &&
                                 parameters["enable_right_ir"].as<bool>());
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "failed to load self-test config: %s",
                 error.what());
    return CallbackReturn::FAILURE;
  }

  if (streams_.empty()) {
    RCLCPP_ERROR(get_logger(), "no enabled image streams found in config");
    return CallbackReturn::FAILURE;
  }

  subscription_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = subscription_group_;
  for (size_t i = 0; i < streams_.size(); ++i) {
    streams_[i].subscription = create_subscription<sensor_msgs::msg::Image>(
        streams_[i].topic, rclcpp::SensorDataQoS(),
        [this, i](sensor_msgs::msg::Image::ConstSharedPtr message) {
          image_callback(i, message);
        },
        options);
  }
  service_ = create_service<std_srvs::srv::Trigger>(
      service_name_,
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        handle_self_test(request, response);
      },
      rmw_qos_profile_services_default, service_group_);
  status_publisher_ =
      create_publisher<std_msgs::msg::Int8>(status_topic_, 10);
  status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_rate_hz_),
      [this] { publish_status(); });
  status_ = 3;
  return CallbackReturn::SUCCESS;
}

CameraSelfCheck::CallbackReturn CameraSelfCheck::on_activate(
    const rclcpp_lifecycle::State&) {
  status_publisher_->on_activate();
  active_ = true;
  return CallbackReturn::SUCCESS;
}

CameraSelfCheck::CallbackReturn CameraSelfCheck::on_deactivate(
    const rclcpp_lifecycle::State&) {
  active_ = false;
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    collecting_ = false;
  }
  sample_cv_.notify_all();
  if (status_publisher_) status_publisher_->on_deactivate();
  status_ = 3;
  return CallbackReturn::SUCCESS;
}

CameraSelfCheck::CallbackReturn CameraSelfCheck::on_cleanup(
    const rclcpp_lifecycle::State& state) {
  on_deactivate(state);
  status_timer_.reset();
  status_publisher_.reset();
  service_.reset();
  for (auto& stream : streams_) stream.subscription.reset();
  streams_.clear();
  subscription_group_.reset();
  service_group_.reset();
  return CallbackReturn::SUCCESS;
}

CameraSelfCheck::CallbackReturn CameraSelfCheck::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  return on_cleanup(state);
}

void CameraSelfCheck::image_callback(
    size_t index, sensor_msgs::msg::Image::ConstSharedPtr) {
  std::lock_guard<std::mutex> lock(sample_mutex_);
  if (!collecting_) return;
  const auto now = monotonic_raw_nanoseconds();
  if (streams_[index].frames == 0) {
    streams_[index].first_timestamp_ns = now;
  }
  streams_[index].last_timestamp_ns = now;
  ++streams_[index].frames;
  sample_cv_.notify_all();
}

void CameraSelfCheck::publish_status() {
  if (status_publisher_ && status_publisher_->is_activated()) {
    std_msgs::msg::Int8 message;
    message.data = status_;
    status_publisher_->publish(message);
  }
}

void CameraSelfCheck::handle_self_test(
    std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!active_) {
    response->message = "Node is not in ACTIVE state";
    return;
  }
  bool expected = false;
  if (!test_running_.compare_exchange_strong(expected, true)) {
    response->message = "Orbbec self-test is already running";
    return;
  }

  status_ = 0;
  std::unique_lock<std::mutex> lock(sample_mutex_);
  for (auto& stream : streams_) stream.frames = 0;
  collecting_ = true;
  const auto all_started = [this] {
    for (const auto& stream : streams_) {
      if (stream.frames == 0) return false;
    }
    return true;
  };
  if (!sample_cv_.wait_for(
          lock, std::chrono::duration<double>(first_frame_timeout_sec_),
          all_started)) {
    collecting_ = false;
    status_ = 2;
    response->message =
        "One or more configured streams did not receive frames";
    test_running_ = false;
    return;
  }

  const auto sample_end = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(test_duration_sec_);
  sample_cv_.wait_until(lock, sample_end, [this] { return !active_.load(); });
  collecting_ = false;
  if (!active_) {
    status_ = 3;
    response->message = "Node left ACTIVE state during self-test";
    test_running_ = false;
    return;
  }

  bool success = true;
  std::ostringstream message;
  for (const auto& stream : streams_) {
    const double elapsed =
        static_cast<double>(stream.last_timestamp_ns -
                            stream.first_timestamp_ns) /
        1e9;
    const double fps = stream.frames > 1 && elapsed > 0.0
                           ? (stream.frames - 1) / elapsed
                           : 0.0;
    const bool pass = fps >= target_fps_ * (1.0 - fps_tolerance_) &&
                      fps <= target_fps_ * (1.0 + fps_tolerance_);
    success = success && pass;
    message << stream.topic << " fps=" << std::fixed << std::setprecision(2)
            << fps << "; ";
  }
  status_ = success ? 1 : 2;
  response->success = success;
  response->message = message.str();
  test_running_ = false;
}

}  // namespace orbbec_camera_lifecycle

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<orbbec_camera_lifecycle::CameraSelfCheck>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                     2);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
