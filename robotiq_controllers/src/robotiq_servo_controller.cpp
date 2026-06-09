#include "robotiq_controllers/robotiq_servo_controller.hpp"

#include <algorithm>
#include <cmath>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace robotiq_controllers
{
namespace
{
constexpr std::size_t kExpectedCommandSize = 3U;
constexpr double kMinAlpha = 0.0;
constexpr double kMaxAlpha = 1.0;
constexpr double kAdaptiveDiffForFullAlpha = 0.2;

void write_command_interface(hardware_interface::LoanedCommandInterface& command_interface, double value)
{
  const bool write_ok = command_interface.set_value(value);
  (void)write_ok;
}
}  // namespace

controller_interface::CallbackReturn RobotiqServoController::on_init()
{
  auto_declare<std::string>("joint", "robotiq_85_left_knuckle_joint");
  auto_declare<std::string>("command_topic", "~/commands");
  auto_declare<std::string>("openness_topic", "~/openness");
  auto_declare<double>("smooth_alpha", 0.3);
  auto_declare<double>("min_position", 0.0);
  auto_declare<double>("max_position", 0.792);
  return controller_interface::CallbackReturn::SUCCESS;
}
controller_interface::InterfaceConfiguration RobotiqServoController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  config.names.emplace_back(joint_name_ + "/set_gripper_max_velocity");
  config.names.emplace_back(joint_name_ + "/set_gripper_max_effort");
  return config;
}

controller_interface::InterfaceConfiguration RobotiqServoController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.emplace_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  return config;
}

controller_interface::CallbackReturn
RobotiqServoController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
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
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create Robotiq command subscriber: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  ServoCommand initial_command;
  initial_command.position = min_position_;
  initial_command.max_velocity = 0.0;
  initial_command.max_effort = 0.0;
  initial_command.valid = false;
  rt_command_buffer_.writeFromNonRT(initial_command);
  active_command_ = initial_command;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RobotiqServoController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
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
  const double initial_velocity = std::abs(command_interfaces_[MAX_VELOCITY_CMD].get_optional().value_or(0.0));
  const double initial_effort = std::max(command_interfaces_[MAX_EFFORT_CMD].get_optional().value_or(0.0), 0.0);
  active_command_.position = initial_position;
  active_command_.max_velocity = initial_velocity;
  active_command_.max_effort = initial_effort;
  active_command_.valid = false;
  filtered_position_non_rt_ = initial_position;
  has_filtered_position_non_rt_ = true;
  rt_command_buffer_.writeFromNonRT(active_command_);

  write_command_interface(command_interfaces_[POSITION_CMD], initial_position);
  write_command_interface(command_interfaces_[MAX_VELOCITY_CMD], initial_velocity);
  write_command_interface(command_interfaces_[MAX_EFFORT_CMD], initial_effort);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RobotiqServoController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  command_sub_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RobotiqServoController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  command_sub_.reset();
  realtime_openness_pub_.reset();
  openness_pub_.reset();
  has_filtered_position_non_rt_ = false;
  active_command_ = ServoCommand{};
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RobotiqServoController::update(const rclcpp::Time& /*time*/,
                                                                 const rclcpp::Duration& /*period*/)
{
  const auto* latest_command = rt_command_buffer_.readFromRT();
  if (latest_command != nullptr)
  {
    active_command_ = *latest_command;
  }

  const double next_position = clamp_position(active_command_.position);
  const double commanded_velocity = std::abs(active_command_.max_velocity);
  const double commanded_effort = std::max(active_command_.max_effort, 0.0);

  write_command_interface(command_interfaces_[POSITION_CMD], next_position);
  write_command_interface(command_interfaces_[MAX_VELOCITY_CMD], commanded_velocity);
  write_command_interface(command_interfaces_[MAX_EFFORT_CMD], commanded_effort);

  const double current_position = state_interfaces_[POSITION_STATE].get_optional().value_or(min_position_);
  if (realtime_openness_pub_)
  {
    const double denominator = std::max(1e-6, max_position_ - min_position_);
    double normalized = (current_position - min_position_) / denominator;
    normalized = std::clamp(normalized, 0.0, 1.0);

    std_msgs::msg::Float64 msg;
    msg.data = 1.0 - normalized;
    realtime_openness_pub_->try_publish(msg);
  }

  return controller_interface::return_type::OK;
}

bool RobotiqServoController::read_parameters()
{
  joint_name_ = get_node()->get_parameter("joint").as_string();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  openness_topic_ = get_node()->get_parameter("openness_topic").as_string();
  smooth_alpha_ = get_node()->get_parameter("smooth_alpha").as_double();
  min_position_ = get_node()->get_parameter("min_position").as_double();
  max_position_ = get_node()->get_parameter("max_position").as_double();

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
  smooth_alpha_ = std::clamp(smooth_alpha_, kMinAlpha, kMaxAlpha);
  return true;
}

void RobotiqServoController::command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != kExpectedCommandSize)
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Expected %zu Robotiq command values [position, max_velocity, max_effort], got %zu.",
                         kExpectedCommandSize, msg->data.size());
    return;
  }

  const double requested_position = msg->data[0];
  const double requested_velocity = msg->data[1];
  const double requested_effort = msg->data[2];
  if (!std::isfinite(requested_position) || !std::isfinite(requested_velocity) || !std::isfinite(requested_effort))
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Ignoring non-finite Robotiq servo command.");
    return;
  }

  const double clamped_position = clamp_position(requested_position);
  if (!has_filtered_position_non_rt_)
  {
    filtered_position_non_rt_ = clamped_position;
    has_filtered_position_non_rt_ = true;
  }
  else
  {
    const double diff = std::abs(clamped_position - filtered_position_non_rt_);
    const double adaptive_alpha = std::clamp(diff / kAdaptiveDiffForFullAlpha, smooth_alpha_, kMaxAlpha);
    filtered_position_non_rt_ = adaptive_alpha * clamped_position + (1.0 - adaptive_alpha) * filtered_position_non_rt_;
  }

  ServoCommand next_command;
  next_command.position = clamp_position(filtered_position_non_rt_);
  next_command.max_velocity = std::abs(requested_velocity);
  next_command.max_effort = std::max(requested_effort, 0.0);
  next_command.valid = true;
  rt_command_buffer_.writeFromNonRT(next_command);
}

double RobotiqServoController::clamp_position(double position) const
{
  return std::clamp(position, min_position_, max_position_);
}

}  // namespace robotiq_controllers

PLUGINLIB_EXPORT_CLASS(robotiq_controllers::RobotiqServoController, controller_interface::ControllerInterface)