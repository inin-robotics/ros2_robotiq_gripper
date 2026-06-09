#include "robotiq_controllers/robotiq_chunk_controller.hpp"

#include <algorithm>
#include <cmath>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace robotiq_controllers
{
namespace
{
constexpr double kMinAlpha = 0.0;
constexpr double kMaxAlpha = 1.0;
constexpr double kMinOpenness = 0.0;
constexpr double kMaxOpenness = 1.0;
constexpr double kAdaptiveDiffForFullAlpha = 0.2;

void write_command_interface(hardware_interface::LoanedCommandInterface& command_interface, double value)
{
  const bool write_ok = command_interface.set_value(value);
  (void)write_ok;
}
}  // namespace

controller_interface::CallbackReturn RobotiqChunkController::on_init()
{
  auto_declare<std::string>("joint", "robotiq_85_left_knuckle_joint");
  auto_declare<std::string>("command_topic", "~/commands");
  auto_declare<std::string>("openness_topic", "~/openness");
  auto_declare<double>("smooth_alpha", 0.3);
  auto_declare<double>("min_position", 0.0);
  auto_declare<double>("max_position", 0.792);
  auto_declare<double>("max_velocity", 0.15);
  auto_declare<double>("max_effort", 1.0);
  auto_declare<double>("chunk_dt", 0.0333);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RobotiqChunkController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  config.names.emplace_back(joint_name_ + "/set_gripper_max_velocity");
  config.names.emplace_back(joint_name_ + "/set_gripper_max_effort");
  return config;
}

controller_interface::InterfaceConfiguration RobotiqChunkController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  return config;
}

controller_interface::CallbackReturn
RobotiqChunkController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!read_parameters())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  try
  {
    command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        command_topic_, rclcpp::SystemDefaultsQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { this->command_callback(msg); });

    openness_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>(openness_topic_, rclcpp::SystemDefaultsQoS());
    realtime_openness_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64>>(openness_pub_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create Robotiq chunk subscriber: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  ChunkCommand initial_command;
  rt_command_buffer_.writeFromNonRT(initial_command);
  active_command_ = initial_command;
  command_sequence_counter_.store(0U, std::memory_order_relaxed);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RobotiqChunkController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (command_interfaces_.size() != 3U)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 3 command interfaces, got %zu.", command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (state_interfaces_.size() != 1U)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 state interface, got %zu.", state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto current_position = state_interfaces_[POSITION_STATE].get_optional().value_or(min_position_);
  const double initial_position =
      clamp_position(command_interfaces_[POSITION_CMD].get_optional().value_or(current_position));
  active_command_ = ChunkCommand{};
  filtered_openness_rt_ = position_to_openness(initial_position);
  has_filtered_openness_rt_ = true;
  rt_command_buffer_.writeFromNonRT(active_command_);

  write_command_interface(command_interfaces_[POSITION_CMD], initial_position);
  write_command_interface(command_interfaces_[MAX_VELOCITY_CMD], std::abs(max_velocity_));
  write_command_interface(command_interfaces_[MAX_EFFORT_CMD], std::max(max_effort_, 0.0));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RobotiqChunkController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  command_sub_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RobotiqChunkController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  command_sub_.reset();
  realtime_openness_pub_.reset();
  openness_pub_.reset();
  active_command_ = ChunkCommand{};
  filtered_openness_rt_ = 0.0;
  has_filtered_openness_rt_ = false;
  command_sequence_counter_.store(0U, std::memory_order_relaxed);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotiqChunkController::update(const rclcpp::Time& time,
                                                                 const rclcpp::Duration& /*period*/)
{
  const auto* latest_command = rt_command_buffer_.readFromRT();
  if (latest_command != nullptr && latest_command->sequence != active_command_.sequence)
  {
    active_command_ = *latest_command;
  }

  double target_openness = filtered_openness_rt_;
  double sampled_openness = 0.0;
  if (sample_command_openness(active_command_, time, &sampled_openness))
  {
    if (!has_filtered_openness_rt_)
    {
      filtered_openness_rt_ = sampled_openness;
      has_filtered_openness_rt_ = true;
    }
    else
    {
      const double diff = std::abs(sampled_openness - filtered_openness_rt_);
      const double adaptive_alpha = std::clamp(diff / kAdaptiveDiffForFullAlpha, smooth_alpha_, kMaxAlpha);
      filtered_openness_rt_ = adaptive_alpha * sampled_openness + (1.0 - adaptive_alpha) * filtered_openness_rt_;
    }
    target_openness = filtered_openness_rt_;
  }
  else if (!has_filtered_openness_rt_)
  {
    const double current_position = state_interfaces_[POSITION_STATE].get_optional().value_or(min_position_);
    filtered_openness_rt_ = position_to_openness(current_position);
    has_filtered_openness_rt_ = true;
    target_openness = filtered_openness_rt_;
  }

  const double next_position = clamp_position(openness_to_position(target_openness));
  write_command_interface(command_interfaces_[POSITION_CMD], next_position);
  write_command_interface(command_interfaces_[MAX_VELOCITY_CMD], std::abs(max_velocity_));
  write_command_interface(command_interfaces_[MAX_EFFORT_CMD], std::max(max_effort_, 0.0));

  const double current_position = state_interfaces_[POSITION_STATE].get_optional().value_or(min_position_);
  if (realtime_openness_pub_)
  {
    std_msgs::msg::Float64 msg;
    msg.data = position_to_openness(current_position);
    realtime_openness_pub_->try_publish(msg);
  }

  return controller_interface::return_type::OK;
}

bool RobotiqChunkController::read_parameters()
{
  joint_name_ = get_node()->get_parameter("joint").as_string();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  openness_topic_ = get_node()->get_parameter("openness_topic").as_string();
  smooth_alpha_ = get_node()->get_parameter("smooth_alpha").as_double();
  min_position_ = get_node()->get_parameter("min_position").as_double();
  max_position_ = get_node()->get_parameter("max_position").as_double();
  max_velocity_ = get_node()->get_parameter("max_velocity").as_double();
  max_effort_ = get_node()->get_parameter("max_effort").as_double();
  chunk_dt_ = get_node()->get_parameter("chunk_dt").as_double();

  if (joint_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joint' must not be empty.");
    return false;
  }
  if (command_topic_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'command_topic' must not be empty.");
    return false;
  }
  if (openness_topic_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'openness_topic' must not be empty.");
    return false;
  }
  if (min_position_ > max_position_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "min_position must be <= max_position.");
    return false;
  }
  if (max_velocity_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "max_velocity must be >= 0.");
    return false;
  }
  if (max_effort_ < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "max_effort must be >= 0.");
    return false;
  }
  if (chunk_dt_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "chunk_dt must be positive.");
    return false;
  }

  smooth_alpha_ = std::clamp(smooth_alpha_, kMinAlpha, kMaxAlpha);
  return true;
}

void RobotiqChunkController::command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!msg)
  {
    return;
  }
  if (msg->data.empty())
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Ignoring empty Robotiq chunk command.");
    return;
  }

  auto openness_samples = std::make_shared<std::vector<double>>(msg->data.size(), 0.0);
  for (std::size_t i = 0; i < msg->data.size(); ++i)
  {
    if (!std::isfinite(msg->data[i]))
    {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Ignoring non-finite Robotiq chunk command.");
      return;
    }
    (*openness_samples)[i] = clamp_openness(msg->data[i]);
  }

  ChunkCommand next_command;
  next_command.openness_samples = openness_samples;
  next_command.stamp_ns = get_node()->get_clock()->now().nanoseconds();
  next_command.sequence = command_sequence_counter_.fetch_add(1U, std::memory_order_relaxed) + 1U;
  next_command.valid = true;
  rt_command_buffer_.writeFromNonRT(next_command);
}

bool RobotiqChunkController::sample_command_openness(const ChunkCommand& command, const rclcpp::Time& time,
                                                     double* openness) const
{
  if (openness == nullptr || !command.valid || !command.openness_samples || command.openness_samples->empty())
  {
    return false;
  }

  const auto& openness_samples = *command.openness_samples;
  if (openness_samples.size() == 1U)
  {
    *openness = openness_samples.front();
    return true;
  }

  const int64_t elapsed_ns = time.nanoseconds() - command.stamp_ns;
  const double t_elapsed = std::max(0.0, static_cast<double>(elapsed_ns) * 1.0e-9);
  const double segment_index_real = std::floor(t_elapsed / chunk_dt_);
  const std::size_t segment_index = static_cast<std::size_t>(std::max(0.0, segment_index_real));
  if (segment_index >= openness_samples.size() - 1U)
  {
    *openness = openness_samples.back();
    return true;
  }

  const double t_segment = static_cast<double>(segment_index) * chunk_dt_;
  const double alpha = std::clamp((t_elapsed - t_segment) / chunk_dt_, kMinAlpha, kMaxAlpha);
  const double openness_0 = openness_samples[segment_index];
  const double openness_1 = openness_samples[segment_index + 1U];
  *openness = openness_0 + (openness_1 - openness_0) * alpha;
  return true;
}

double RobotiqChunkController::clamp_position(double position) const
{
  return std::clamp(position, min_position_, max_position_);
}

double RobotiqChunkController::clamp_openness(double openness) const
{
  return std::clamp(openness, kMinOpenness, kMaxOpenness);
}

double RobotiqChunkController::openness_to_position(double openness) const
{
  const double normalized = clamp_openness(openness);
  return min_position_ + (1.0 - normalized) * (max_position_ - min_position_);
}

double RobotiqChunkController::position_to_openness(double position) const
{
  const double denominator = std::max(1e-6, max_position_ - min_position_);
  const double normalized = (clamp_position(position) - min_position_) / denominator;
  return clamp_openness(1.0 - normalized);
}

}  // namespace robotiq_controllers

PLUGINLIB_EXPORT_CLASS(robotiq_controllers::RobotiqChunkController, controller_interface::ControllerInterface)
