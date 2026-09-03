#pragma once

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace orbbec_camera_lifecycle {

class CameraSelfCheck : public rclcpp_lifecycle::LifecycleNode {
 public:
  explicit CameraSelfCheck(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

 private:
  struct Stream {
    std::string topic;
    uint64_t frames{0};
    uint64_t first_timestamp_ns{0};
    uint64_t last_timestamp_ns{0};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;
  };

  void image_callback(size_t index,
                      sensor_msgs::msg::Image::ConstSharedPtr message);
  void handle_self_test(
      std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void publish_status();

  std::string config_file_;
  std::string service_name_ = "orbbec_self_test";
  std::string status_topic_ = "orbbec_self_test_status";
  double target_fps_ = 30.0;
  double fps_tolerance_ = 0.1;
  double test_duration_sec_ = 2.0;
  double first_frame_timeout_sec_ = 2.0;
  double status_rate_hz_ = 1.0;

  std::atomic<int8_t> status_{3};
  std::atomic_bool active_{false};
  std::atomic_bool test_running_{false};
  std::mutex sample_mutex_;
  std::condition_variable sample_cv_;
  bool collecting_ = false;
  std::vector<Stream> streams_;

  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int8>::SharedPtr
      status_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace orbbec_camera_lifecycle
