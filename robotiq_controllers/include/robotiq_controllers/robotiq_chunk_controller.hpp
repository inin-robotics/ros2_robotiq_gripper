#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace robotiq_controllers
{
class RobotiqChunkController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  struct ChunkCommand
  {
    std::shared_ptr<const std::vector<double>> openness_samples;
    int64_t stamp_ns = 0;
    uint64_t sequence = 0;
    bool valid = false;
  };

  bool read_parameters();
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  bool sample_command_openness(const ChunkCommand& command, const rclcpp::Time& time, double* openness) const;
  double clamp_position(double position) const;
  double clamp_openness(double openness) const;
  double openness_to_position(double openness) const;
  double position_to_openness(double position) const;

  enum CommandInterfaces
  {
    POSITION_CMD = 0,
    MAX_VELOCITY_CMD = 1,
    MAX_EFFORT_CMD = 2,
  };

  enum StateInterfaces
  {
    POSITION_STATE = 0,
  };

  std::string joint_name_;
  std::string command_topic_;
  std::string openness_topic_;
  double smooth_alpha_ = 0.3;
  double min_position_ = 0.0;
  double max_position_ = 0.8;
  double max_velocity_ = 0.15;
  double max_effort_ = 1.0;
  double chunk_dt_ = 0.0333;
  double filtered_openness_rt_ = 0.0;
  bool has_filtered_openness_rt_ = false;
  std::atomic<uint64_t> command_sequence_counter_{ 0U };

  realtime_tools::RealtimeBuffer<ChunkCommand> rt_command_buffer_;
  ChunkCommand active_command_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr openness_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>> realtime_openness_pub_;
};
}  // namespace robotiq_controllers
