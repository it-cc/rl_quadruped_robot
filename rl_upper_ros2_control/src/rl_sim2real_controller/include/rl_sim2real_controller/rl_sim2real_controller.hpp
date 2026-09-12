#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "onnxruntime_cxx_api.h"

namespace rl_sim2real
{
#ifndef RL_SIM2REAL_TEST_POLICY
#define RL_SIM2REAL_TEST_POLICY 0
#endif
#ifndef RL_TEST_LOG_PERIOD_MS
#define RL_TEST_LOG_PERIOD_MS 1000
#endif

class RL_Sim2RealController
    : public controller_interface::ChainableControllerInterface
{
 public:
  RL_Sim2RealController() = default;
  ~RL_Sim2RealController() override = default;

  controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  controller_interface::return_type update_and_write_commands(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  std::vector<hardware_interface::CommandInterface>
  on_export_reference_interfaces() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 protected:
  std::vector<hardware_interface::StateInterface::SharedPtr>
  on_export_state_interfaces_list() override;

 private:
  static constexpr std::size_t kJointCount = 12;
  static constexpr std::size_t kObservationSize = 26;
  static constexpr std::size_t kActionSize = 12;
  static constexpr float kPhasePeriodSeconds = 0.4F;
  static constexpr float kCommandDeadband = 0.1F;

  static bool finite(const std::array<float, kObservationSize>& values);
  bool initialize_policy();
  bool read_state(std::array<float, kObservationSize>& observation) const;
  bool run_policy(const std::array<float, kObservationSize>& observation,
                  std::array<float, kActionSize>& action);
  void write_default_position();
  bool is_standing_command(
      const std::array<float, kObservationSize>& observation) const;

  std::string policy_path_;
  double control_period_seconds_{0.02};
  float action_scale_{0.25F};
  float standing_linear_velocity_deadband_{0.02F};
  float standing_angular_velocity_deadband_{0.02F};
  rclcpp::Time last_policy_time_;
  rclcpp::Time last_debug_log_time_;
  double phase_time_seconds_{0.0};
  bool policy_ready_{false};
  std::array<float, kJointCount> default_joint_position_{};
  std::array<float, kActionSize> last_action_{};
  std::array<double, kJointCount> exported_joint_positions_{};

  std::unique_ptr<Ort::Env> onnx_environment_;
  Ort::SessionOptions onnx_session_options_;
  std::unique_ptr<Ort::Session> onnx_session_;
  Ort::AllocatorWithDefaultOptions onnx_allocator_;
  std::string onnx_input_name_;
  std::string onnx_output_name_;
};
}  // namespace rl_sim2real
