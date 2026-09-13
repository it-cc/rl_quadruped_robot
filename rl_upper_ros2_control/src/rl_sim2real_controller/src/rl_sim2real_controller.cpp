#include <algorithm>
#include <cmath>
#include <hardware_interface/introspection.hpp>
#include <limits>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"
#include "rl_sim2real_controller/rl_sim2real_controller.hpp"

namespace rl_sim2real
{
namespace
{
constexpr std::array<const char*, 12> kJointNames = {
    "front_left_hip_joint",    "front_left_thigh_joint",
    "front_left_calf_joint",   "front_right_hip_joint",
    "front_right_thigh_joint", "front_right_calf_joint",
    "back_left_hip_joint",     "back_left_thigh_joint",
    "back_left_calf_joint",    "back_right_hip_joint",
    "back_right_thigh_joint",  "back_right_calf_joint"};
constexpr std::array<float, 12> kDefaultJointPosition = {
    0.0, 0.6151, -0.9065, 0.0, 0.6151, -0.9065,
    0.0, 0.6519, -0.9709, 0.0, 0.6519, -0.9709};
}  // namespace

controller_interface::InterfaceConfiguration
RL_Sim2RealController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const char* joint : kJointNames)
  {
    configuration.names.emplace_back(std::string(joint) + "/" +
                                     hardware_interface::HW_IF_POSITION);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
RL_Sim2RealController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  constexpr std::array<const char*, 12> state_names = {
      "imu/orientation.x",         "imu/orientation.y",
      "imu/orientation.z",         "imu/angular_velocity.x",
      "imu/angular_velocity.y",    "imu/angular_velocity.z",
      "imu/linear_acceleration.x", "imu/linear_acceleration.y",
      "imu/linear_acceleration.z", "remote_control/linear.x",
      "remote_control/linear.y",   "remote_control/angular.z"};
  configuration.names.assign(state_names.begin(), state_names.end());
  return configuration;
}

controller_interface::return_type
RL_Sim2RealController::update_reference_from_subscribers(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type
RL_Sim2RealController::update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& /* period */)
{
  if (!policy_ready_)
  {
    return controller_interface::return_type::ERROR;
  }

  if (last_policy_time_.nanoseconds() != 0 &&
      (time - last_policy_time_).seconds() < control_period_seconds_)
  {
    return controller_interface::return_type::OK;
  }

  std::array<float, kObservationSize> observation{};
  if (!read_state(observation))
  {
    write_default_position();
    return controller_interface::return_type::ERROR;
  }

  if (is_standing_command(observation))
  {
    last_action_.fill(0.0F);
    write_default_position();
    last_policy_time_ = time;
    phase_time_seconds_ += control_period_seconds_;
    RCLCPP_INFO_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Standing mode: command is inside deadband, holding default pose");
    return controller_interface::return_type::OK;
  }

  std::array<float, kActionSize> action{};
  if (!run_policy(observation, action))
  {
    write_default_position();
    return controller_interface::return_type::ERROR;
  }

  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    const double target = static_cast<double>(default_joint_position_[index]) +
                          static_cast<double>(action[index] * action_scale_);
    exported_joint_positions_[index] = target;
    if (!command_interfaces_[index].set_value(target))
    {
      return controller_interface::return_type::ERROR;
    }
  }

  last_action_ = action;
  last_policy_time_ = time;
  phase_time_seconds_ += control_period_seconds_;

  RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "\n"
      "========== RL Action / Target ==========\n"
      "FL hip    action = % .4f   target = % .4f\n"
      "FL thigh  action = % .4f   target = % .4f\n"
      "FL calf   action = % .4f   target = % .4f\n"
      "FR hip    action = % .4f   target = % .4f\n"
      "FR thigh  action = % .4f   target = % .4f\n"
      "FR calf   action = % .4f   target = % .4f\n"
      "BL hip    action = % .4f   target = % .4f\n"
      "BL thigh  action = % .4f   target = % .4f\n"
      "BL calf   action = % .4f   target = % .4f\n"
      "BR hip    action = % .4f   target = % .4f\n"
      "BR thigh  action = % .4f   target = % .4f\n"
      "BR calf   action = % .4f   target = % .4f\n"
      "========================================",
      action[0], exported_joint_positions_[0], action[1],
      exported_joint_positions_[1], action[2], exported_joint_positions_[2],
      action[3], exported_joint_positions_[3], action[4],
      exported_joint_positions_[4], action[5], exported_joint_positions_[5],
      action[6], exported_joint_positions_[6], action[7],
      exported_joint_positions_[7], action[8], exported_joint_positions_[8],
      action[9], exported_joint_positions_[9], action[10],
      exported_joint_positions_[10], action[11], exported_joint_positions_[11]);

  return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface>
RL_Sim2RealController::on_export_reference_interfaces()
{
  return {};
}

std::vector<hardware_interface::StateInterface::SharedPtr>
RL_Sim2RealController::on_export_state_interfaces_list()
{
  std::vector<hardware_interface::StateInterface::SharedPtr> interfaces;
  interfaces.reserve(kJointCount);
  for (std::size_t index = 0; index < kJointCount; ++index)
  {
    interfaces.emplace_back(
        std::make_shared<hardware_interface::StateInterface>(
            "rl_sim2real_controller", "joint_position_" + std::to_string(index),
            &exported_joint_positions_[index]));
  }
  return interfaces;
}

controller_interface::CallbackReturn RL_Sim2RealController::on_init()
{
  try
  {
    policy_path_ = get_node()->declare_parameter<std::string>("policy_path",
                                                              "/error/path");
    control_period_seconds_ =
        get_node()->declare_parameter<double>("control_period", 0.02);
    action_scale_ = static_cast<float>(
        get_node()->declare_parameter<double>("action_scale", 0.25));
    standing_linear_velocity_deadband_ =
        static_cast<float>(get_node()->declare_parameter<double>(
            "standing_linear_velocity_deadband", 0.1));
    standing_angular_velocity_deadband_ =
        static_cast<float>(get_node()->declare_parameter<double>(
            "standing_angular_velocity_deadband", 0.1));
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid policy parameter: %s",
                 error.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (control_period_seconds_ <= 0.0 || action_scale_ <= 0.0F ||
      standing_linear_velocity_deadband_ < 0.0F ||
      standing_angular_velocity_deadband_ < 0.0F)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "control_period and action_scale must be > 0; standing "
                 "deadbands must be >= 0");
    return controller_interface::CallbackReturn::ERROR;
  }
  default_joint_position_ = kDefaultJointPosition;
  last_action_.fill(0.0F);
  exported_joint_positions_.fill(0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RL_Sim2RealController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!initialize_policy()) return controller_interface::CallbackReturn::ERROR;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RL_Sim2RealController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  last_action_.fill(0.0F);
  phase_time_seconds_ = 0.0;
  last_policy_time_ =
      rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  write_default_position();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RL_Sim2RealController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  write_default_position();
  return controller_interface::CallbackReturn::SUCCESS;
}

bool RL_Sim2RealController::finite(
    const std::array<float, kObservationSize>& values)
{
  return std::all_of(values.begin(), values.end(),
                     [](float value) { return std::isfinite(value); });
}

bool RL_Sim2RealController::read_state(
    std::array<float, kObservationSize>& observation) const
{
  if (state_interfaces_.size() != 12) return false;
  const auto value = [this](std::size_t index)
  {
    const auto state = state_interfaces_[index].get_optional<double>();
    return state.has_value() ? static_cast<float>(state.value())
                             : std::numeric_limits<float>::quiet_NaN();
  };

  const float roll = value(0);
  const float pitch = value(1);
  [[maybe_unused]] const float yaw = value(2);
  const float angular_velocity_x = value(3);
  const float angular_velocity_y = value(4);
  const float angular_velocity_z = value(5);
  const float linear_acceleration_x = value(6);
  const float linear_acceleration_y = value(7);
  const float linear_acceleration_z = value(8);
  const float remote_control_linear_x = value(9);
  const float remote_control_linear_y = value(10);
  const float remote_control_angular_z = value(11);
  const bool standing_command =
      std::sqrt(remote_control_linear_x * remote_control_linear_x +
                remote_control_linear_y * remote_control_linear_y +
                remote_control_angular_z * remote_control_angular_z) <
      kCommandDeadband;
  const float phase = static_cast<float>(
      std::fmod(phase_time_seconds_, static_cast<double>(kPhasePeriodSeconds)) /
      static_cast<double>(kPhasePeriodSeconds));

  // Keep this order identical to the actor observation terms in training.
  observation[0] = angular_velocity_x;
  observation[1] = angular_velocity_y;
  observation[2] = angular_velocity_z;
  observation[3] = std::sin(pitch);
  observation[4] = -std::sin(roll) * std::cos(pitch);
  observation[5] = -std::cos(roll) * std::cos(pitch);
  observation[6] = remote_control_linear_x;
  observation[7] = remote_control_linear_y;
  observation[8] = remote_control_angular_z;
  if (standing_command)
  {
    observation[9] = 0.0F;
    observation[10] = 0.0F;
  }
  else
  {
    observation[9] = std::sin(phase * 2.0F * 3.14159265358979323846F);
    observation[10] = std::cos(phase * 2.0F * 3.14159265358979323846F);
  }
  std::copy(last_action_.begin(), last_action_.end(), observation.begin() + 11);
  observation[23] = linear_acceleration_x;
  observation[24] = linear_acceleration_y;
  observation[25] = linear_acceleration_z;

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                       "\n"
                       "========== RL Observation ==========\n"
                       "angular_velocity_x   = %.4f\n"
                       "angular_velocity_y   = %.4f\n"
                       "angular_velocity_z   = %.4f\n"
                       "gravity_projection_x = %.4f\n"
                       "gravity_projection_y = %.4f\n"
                       "gravity_projection_z = %.4f\n"
                       "phase_sin           = %.4f\n"
                       "phase_cos           = %.4f\n"
                       "command_linear_x     = %.4f\n"
                       "command_linear_y     = %.4f\n"
                       "command_angular_z    = %.4f\n"
                       "linear_acceleration_x = %.4f\n"
                       "linear_acceleration_y = %.4f\n"
                       "linear_acceleration_z = %.4f\n"
                       "====================================",
                       observation[0], observation[1], observation[2],
                       observation[3], observation[4], observation[5],
                       observation[9], observation[10], observation[6],
                       observation[7], observation[8], observation[23],
                       observation[24], observation[25]);

  return finite(observation);
}

bool RL_Sim2RealController::initialize_policy()
{
  try
  {
    onnx_environment_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING,
                                                   "rl_sim2real_controller");
    onnx_session_options_.SetIntraOpNumThreads(1);
    onnx_session_options_.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    onnx_session_ = std::make_unique<Ort::Session>(
        *onnx_environment_, policy_path_.c_str(), onnx_session_options_);
    if (onnx_session_->GetInputCount() != 1 ||
        onnx_session_->GetOutputCount() != 1)
    {
      throw std::runtime_error("policy must have one input and one output");
    }
    const auto input_shape = onnx_session_->GetInputTypeInfo(0)
                                 .GetTensorTypeAndShapeInfo()
                                 .GetShape();
    const auto output_shape = onnx_session_->GetOutputTypeInfo(0)
                                  .GetTensorTypeAndShapeInfo()
                                  .GetShape();
    if (input_shape.size() != 2 || input_shape[0] != 1 ||
        input_shape[1] != static_cast<int64_t>(kObservationSize) ||
        output_shape.size() != 2 || output_shape[0] != 1 ||
        output_shape[1] != static_cast<int64_t>(kActionSize))
    {
      throw std::runtime_error(
          "policy dimensions must be input [1,26], output [1,12]");
    }
    auto input_name = onnx_session_->GetInputNameAllocated(0, onnx_allocator_);
    auto output_name =
        onnx_session_->GetOutputNameAllocated(0, onnx_allocator_);
    onnx_input_name_ = input_name.get();
    onnx_output_name_ = output_name.get();
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load policy %s: %s",
                 policy_path_.c_str(), error.what());
    onnx_session_.reset();
    onnx_environment_.reset();
    return false;
  }
  policy_ready_ = true;
  return true;
}

bool RL_Sim2RealController::run_policy(
    const std::array<float, kObservationSize>& observation,
    std::array<float, kActionSize>& action)
{
  try
  {
    std::array<int64_t, 2> shape{1, static_cast<int64_t>(kObservationSize)};
    auto memory_info =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input = Ort::Value::CreateTensor<float>(
        memory_info, const_cast<float*>(observation.data()), observation.size(),
        shape.data(), shape.size());
    const char* input_names[] = {onnx_input_name_.c_str()};
    const char* output_names[] = {onnx_output_name_.c_str()};
    auto outputs = onnx_session_->Run(Ort::RunOptions{nullptr}, input_names,
                                      &input, 1, output_names, 1);
    if (outputs.size() != 1 || !outputs[0].IsTensor()) return false;
    const auto shape_out = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
    if (shape_out.size() != 2 || shape_out[0] != 1 || shape_out[1] != 12)
      return false;
    const float* output = outputs[0].GetTensorData<float>();
    for (std::size_t index = 0; index < kActionSize; ++index)
    {
      if (!std::isfinite(output[index])) return false;
      action[index] = output[index];
    }
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          1000, "Policy inference failed: %s", error.what());
    return false;
  }
  return true;
}

void RL_Sim2RealController::write_default_position()
{
  for (std::size_t index = 0;
       index < kJointCount && index < command_interfaces_.size(); ++index)
  {
    exported_joint_positions_[index] = default_joint_position_[index];
    (void)command_interfaces_[index].set_value(
        static_cast<double>(default_joint_position_[index]));
  }
}

bool RL_Sim2RealController::is_standing_command(
    const std::array<float, kObservationSize>& observation) const
{
  return std::abs(observation[6]) <= standing_linear_velocity_deadband_ &&
         std::abs(observation[7]) <= standing_linear_velocity_deadband_ &&
         std::abs(observation[8]) <= standing_angular_velocity_deadband_;
}
}  // namespace rl_sim2real

PLUGINLIB_EXPORT_CLASS(rl_sim2real::RL_Sim2RealController,
                       controller_interface::ChainableControllerInterface)
