#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rl_sim2real_msgs/msg/joint_command.hpp"
#include "rl_sim2real_msgs/msg/robot_state.hpp"

namespace rl_sim2real
{

class RL_Sim2RealHardwareInterface
    : public hardware_interface::SystemInterface
{
 public:
  RL_Sim2RealHardwareInterface() = default;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params)
      override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

 private:
  static constexpr std::size_t kJointCount = 12;
  static constexpr std::size_t kRobotVelocityCount = 3;
  static constexpr std::size_t kImuValueCount = 9;

  void state_callback(const rl_sim2real_msgs::msg::RobotState::SharedPtr msg);
  std::string hardware_parameter(const std::string& name,
                                 const std::string& default_value) const;
  bool validate_hardware_info() const;

  // ROS2 node and communication
  rclcpp::Node::SharedPtr node_;
  std::string node_name_{"rl_hardware_interface"};
  rclcpp::Publisher<rl_sim2real_msgs::msg::JointCommand>::SharedPtr
      command_publisher_;
  rclcpp::Subscription<rl_sim2real_msgs::msg::RobotState>::SharedPtr
      state_subscriber_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  // State data
  std::array<double, kJointCount> joint_commands_{};
  std::array<double, kRobotVelocityCount> robot_velocity_{};
  std::array<double, kImuValueCount> imu_values_{};

  // Communication state
  uint8_t sequence_{0};
  uint8_t last_sent_sequence_{0};
  uint8_t last_received_sequence_{0};
  uint32_t mcu_timestamp_ms_{0};
  uint32_t error_state_{0};
  bool has_feedback_{false};
  std::chrono::steady_clock::time_point last_feedback_time_{};
  std::chrono::steady_clock::time_point last_debug_log_time_{};

};
}  // namespace rl_sim2real
