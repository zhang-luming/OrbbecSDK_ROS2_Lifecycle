#include "orbbec_camera_lifecycle/camera_self_check.hpp"
#include "pudu-base/pdLog/log.h"
#include "nvq_tools/tools/common.h"

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
  // Keep callback groups alive until the executor has stopped and the node is
  // destroyed: an active wait set may still reference their guard conditions.
  subscription_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
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
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Lifecycle configure started from %s",
              state.label().c_str());
  config_file_ = get_parameter("config_file").as_string();
  if (config_file_.empty()) {
    config_file_ = ament_index_cpp::get_package_share_directory(
                       "orbbec_camera_lifecycle") +
                   "/config/cameras.yaml";
  }
  streams_.clear();
  try {
    const auto root = YAML::LoadFile(config_file_);
    const auto self_check = root["self_check"];
    if (self_check && !self_check.IsMap()) {
      RCLCPP_ERROR(get_logger(), "self_check config must be a map");
      RCLCPP_ERROR(get_logger(), "Lifecycle configure failed");
      return CallbackReturn::FAILURE;
    }

    if (self_check) {
      const auto& overrides =
          get_node_parameters_interface()->get_parameter_overrides();
      const auto load_string = [this, &self_check, &overrides](
                                   const char* name) {
        if (self_check[name] && overrides.count(name) == 0) {
          set_parameter(
              rclcpp::Parameter(name, self_check[name].as<std::string>()));
        }
      };
      const auto load_double = [this, &self_check, &overrides](
                                   const char* name) {
        if (self_check[name] && overrides.count(name) == 0) {
          set_parameter(rclcpp::Parameter(name, self_check[name].as<double>()));
        }
      };
      load_string("service_name");
      load_string("status_topic");
      load_double("target_fps");
      load_double("fps_tolerance");
      load_double("test_duration_sec");
      load_double("first_frame_timeout_sec");
      load_double("status_rate_hz");
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
      RCLCPP_ERROR(get_logger(),
                   "self-test timing and FPS parameters are invalid");
      RCLCPP_ERROR(get_logger(), "Lifecycle configure failed");
      return CallbackReturn::FAILURE;
    }

    const auto cameras = root["cameras"];
    if (!cameras || !cameras.IsSequence()) {
      RCLCPP_ERROR(get_logger(), "config must contain a cameras sequence");
      RCLCPP_ERROR(get_logger(), "Lifecycle configure failed");
      return CallbackReturn::FAILURE;
    }

    for (const auto& camera : cameras) {
      if (camera["enabled"] && !camera["enabled"].as<bool>()) continue;
      const auto ns = camera["namespace"]
                          ? camera["namespace"].as<std::string>()
                          : camera["name"].as<std::string>();
      const auto parameters = camera["parameters"];
      const auto add_stream = [this, &ns](const char* name, bool enabled,
                                         const char* transport = nullptr) {
        if (!enabled) return;
        Stream stream;
        stream.topic = "/" + ns + "/" + name + "/image_raw";
        stream.compressed = transport != nullptr;
        if (transport) stream.topic += std::string("/") + transport;
        streams_.push_back(std::move(stream));
      };
      add_stream("color", !parameters || !parameters["enable_color"] ||
                              parameters["enable_color"].as<bool>(), "compressed");
      add_stream("depth", !parameters || !parameters["enable_depth"] ||
                              parameters["enable_depth"].as<bool>(), "compressedDepth");
      add_stream("left_ir", parameters && parameters["enable_left_ir"] &&
                                parameters["enable_left_ir"].as<bool>());
      add_stream("right_ir", parameters && parameters["enable_right_ir"] &&
                                 parameters["enable_right_ir"].as<bool>());
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "failed to load self-test config: %s",
                 error.what());
    RCLCPP_ERROR(get_logger(), "Lifecycle configure failed");
    return CallbackReturn::FAILURE;
  }

  if (streams_.empty()) {
    RCLCPP_ERROR(get_logger(), "no enabled image streams found in config");
    RCLCPP_ERROR(get_logger(), "Lifecycle configure failed");
    return CallbackReturn::FAILURE;
  }

  rclcpp::SubscriptionOptions options;
  options.callback_group = subscription_group_;
  for (size_t i = 0; i < streams_.size(); ++i) {
    // Count compressed arrivals without decoding color or depth images.
    if (streams_[i].compressed) {
      streams_[i].subscription =
          create_subscription<sensor_msgs::msg::CompressedImage>(
              streams_[i].topic, rclcpp::SensorDataQoS(),
              [this, i](sensor_msgs::msg::CompressedImage::ConstSharedPtr) {
                frame_callback(i);
              },
              options);
    } else {
      streams_[i].subscription = create_subscription<sensor_msgs::msg::Image>(
          streams_[i].topic, rclcpp::SensorDataQoS(),
          [this, i](sensor_msgs::msg::Image::ConstSharedPtr) {
            frame_callback(i);
          },
          options);
    }
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
  RCLCPP_INFO(get_logger(), "Lifecycle configure completed: %zu streams",
              streams_.size());
  return CallbackReturn::SUCCESS;
}

CameraSelfCheck::CallbackReturn CameraSelfCheck::on_activate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Lifecycle activate started from %s",
              state.label().c_str());
  status_publisher_->on_activate();
  active_ = true;
  RCLCPP_INFO(get_logger(), "Lifecycle activate completed");
  return CallbackReturn::SUCCESS;
}

CameraSelfCheck::CallbackReturn CameraSelfCheck::on_deactivate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Lifecycle deactivate started from %s",
              state.label().c_str());
  active_ = false;
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    collecting_ = false;
  }
  sample_cv_.notify_all();
  if (status_publisher_) status_publisher_->on_deactivate();
  status_ = 3;
  RCLCPP_INFO(get_logger(), "Lifecycle deactivate completed");
  return CallbackReturn::SUCCESS;
}

CameraSelfCheck::CallbackReturn CameraSelfCheck::on_cleanup(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Lifecycle cleanup started from %s",
              state.label().c_str());
  on_deactivate(state);
  status_timer_.reset();
  status_publisher_.reset();
  service_.reset();
  for (auto& stream : streams_) stream.subscription.reset();
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    streams_.clear();
  }
  // Reuse the node's callback groups on the next configure transition.
  RCLCPP_INFO(get_logger(), "Lifecycle cleanup completed");
  return CallbackReturn::SUCCESS;
}

CameraSelfCheck::CallbackReturn CameraSelfCheck::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Lifecycle shutdown started from %s",
              state.label().c_str());
  const auto result = on_cleanup(state);
  RCLCPP_INFO(get_logger(), "Lifecycle shutdown completed");
  return result;
}

void CameraSelfCheck::frame_callback(size_t index) {
  std::lock_guard<std::mutex> lock(sample_mutex_);
  if (index >= streams_.size()) return;
  auto& stream = streams_[index];
  const auto now = monotonic_raw_nanoseconds();
  ++stream.total_frames;
  stream.last_received_timestamp_ns = now;
  if (!collecting_) return;
  if (stream.frames == 0) {
    stream.first_timestamp_ns = now;
  }
  stream.last_timestamp_ns = now;
  ++stream.frames;
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
    RCLCPP_WARN(get_logger(), "Self-test rejected: %s",
                response->message.c_str());
    return;
  }
  bool expected = false;
  if (!test_running_.compare_exchange_strong(expected, true)) {
    response->message = "Orbbec self-test is already running";
    RCLCPP_WARN(get_logger(), "Self-test rejected: %s",
                response->message.c_str());
    return;
  }

  status_ = 0;
  std::unique_lock<std::mutex> lock(sample_mutex_);
  RCLCPP_INFO(get_logger(),
              "Self-test started: streams=%zu, first_frame_timeout=%.2fs, "
              "sample_duration=%.2fs, target_fps=%.2f, tolerance=%.2f",
              streams_.size(), first_frame_timeout_sec_, test_duration_sec_,
              target_fps_, fps_tolerance_);
  if (!active_) {
    status_ = 3;
    response->message = "Node left ACTIVE state during self-test";
    test_running_ = false;
    lock.unlock();
    RCLCPP_WARN(get_logger(), "Self-test interrupted: %s",
                response->message.c_str());
    return;
  }
  for (auto& stream : streams_) {
    stream.frames = 0;
    stream.first_timestamp_ns = 0;
    stream.last_timestamp_ns = 0;
  }
  collecting_ = true;
  const auto all_started = [this] {
    for (const auto& stream : streams_) {
      if (stream.frames == 0) return false;
    }
    return true;
  };
  sample_cv_.wait_for(lock,
                      std::chrono::duration<double>(first_frame_timeout_sec_),
                      [this, &all_started] {
                        return !active_.load() || all_started();
                      });
  if (!active_) {
    collecting_ = false;
    status_ = 3;
    response->message = "Node left ACTIVE state during self-test";
    test_running_ = false;
    lock.unlock();
    RCLCPP_WARN(get_logger(), "Self-test interrupted: %s",
                response->message.c_str());
    return;
  }
  if (!all_started()) {
    collecting_ = false;
    status_ = 2;
    const auto now_ns = monotonic_raw_nanoseconds();
    std::ostringstream missing;
    std::ostringstream received;
    bool first_missing = true;
    bool first_received = true;
    for (const auto& stream : streams_) {
      if (stream.frames == 0) {
        if (!first_missing) missing << ", ";
        first_missing = false;
        missing << stream.topic;
        if (stream.total_frames == 0) {
          missing << "(never_seen_since_configure)";
        } else {
          const double age_sec =
              static_cast<double>(now_ns - stream.last_received_timestamp_ns) /
              1e9;
          missing << "(last_seen=" << std::fixed << std::setprecision(2)
                  << age_sec << "s_ago,total=" << stream.total_frames << ")";
        }
      } else {
        if (!first_received) received << ", ";
        first_received = false;
        received << stream.topic << "(frames=" << stream.frames << ")";
      }
    }
    std::ostringstream message;
    message << "One or more configured streams did not receive frames "
            << "within " << std::fixed << std::setprecision(2)
            << first_frame_timeout_sec_ << "s: missing=[" << missing.str()
            << "]; received=[" << received.str() << "]";
    response->message = message.str();
    test_running_ = false;
    lock.unlock();
    RCLCPP_ERROR(get_logger(), "Self-test failed: %s",
                 response->message.c_str());
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
    lock.unlock();
    RCLCPP_WARN(get_logger(), "Self-test interrupted: %s",
                response->message.c_str());
    return;
  }

  bool success = true;
  std::ostringstream details;
  const double min_fps = target_fps_ * (1.0 - fps_tolerance_);
  const double max_fps = target_fps_ * (1.0 + fps_tolerance_);
  for (const auto& stream : streams_) {
    const double elapsed =
        static_cast<double>(stream.last_timestamp_ns -
                            stream.first_timestamp_ns) /
        1e9;
    const double fps = stream.frames > 1 && elapsed > 0.0
                           ? (stream.frames - 1) / elapsed
                           : 0.0;
    const bool pass = fps >= min_fps && fps <= max_fps;
    success = success && pass;
    if (details.tellp() > 0) details << "; ";
    details << stream.topic << "(fps=" << std::fixed << std::setprecision(2)
            << fps << ", frames=" << stream.frames
            << ", result=" << (pass ? "PASS" : "FAIL") << ")";
  }
  status_ = success ? 1 : 2;
  response->success = success;
  std::ostringstream message;
  message << (success ? "PASS" : "FPS_OUT_OF_RANGE") << " expected="
          << std::fixed << std::setprecision(2) << min_fps << ".." << max_fps
          << "fps; streams=[" << details.str() << "]";
  response->message = message.str();
  test_running_ = false;
  lock.unlock();
  if (success) {
    RCLCPP_INFO(get_logger(), "Self-test passed: %s",
                response->message.c_str());
  } else {
    RCLCPP_ERROR(get_logger(), "Self-test failed: %s",
                 response->message.c_str());
  }
}

}  // namespace orbbec_camera_lifecycle

int main(int argc, char** argv) {
  auto log = base::initLog("", pudu::tools::Tools::getLogPathFromEnv(),
                           "orbbec_self_test_node", true,
                           ORBBEC_LIFECYCLE_VERSION, "");
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
