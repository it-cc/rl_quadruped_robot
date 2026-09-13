#include <algorithm>
#include <cmath>
#include <cstring>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logging.hpp"
#include "default_joint_constants.hpp"
#include "rl_sim2real_hardware_interface/rl_sim2real_hardware_interface.hpp"

namespace rl_sim2real
{

namespace
{
constexpr double kStandardGravity = 9.80665;
}

hardware_interface::CallbackReturn RL_Sim2RealHardwareInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_commands_ = rl_quadruped::kDefaultJointPositionDouble;
  robot_velocity_.fill(0.0);
  imu_values_.fill(0.0);
  imu_values_[8] = kStandardGravity;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RL_Sim2RealHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring interface...");

  // Create ROS2 node
  node_ = std::make_shared<rclcpp::Node>(node_name_);

  // Create publisher for joint commands
  command_publisher_ =
      node_->create_publisher<rl_sim2real_msgs::msg::JointCommand>(
          "joint_command", 10);

  // Create subscription for robot state
  state_subscriber_ =
      node_->create_subscription<rl_sim2real_msgs::msg::RobotState>(
          "robot_state", 10,
          [this](const rl_sim2real_msgs::msg::RobotState::SharedPtr msg)
          { state_callback(msg); });

  // Create executor and spin in separate thread
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger(node_name_),
              "micro-ROS interface configured successfully");
  RCLCPP_INFO(rclcpp::get_logger(node_name_), "  Publishing to: joint_command");
  RCLCPP_INFO(rclcpp::get_logger(node_name_), "  Subscribing to: robot_state");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RL_Sim2RealHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(node_name_),
              "Activating micro-ROS interface...");

  sequence_ = 0;
  last_sent_sequence_ = 0;
  last_received_sequence_ = 0;
  has_feedback_ = false;
  error_state_ = 0;

  last_feedback_time_ = std::chrono::steady_clock::now();
  last_debug_log_time_ = std::chrono::steady_clock::now();
  next_command_publish_time_ = std::chrono::steady_clock::now();

  joint_commands_ = rl_quadruped::kDefaultJointPositionDouble;

  RCLCPP_INFO(rclcpp::get_logger(node_name_), "micro-ROS interface activated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RL_Sim2RealHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(node_name_),
              "Deactivating micro-ROS interface...");

  // Stop executor
  if (executor_)
  {
    executor_->cancel();
  }
  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }

  RCLCPP_INFO(rclcpp::get_logger(node_name_),
              "micro-ROS interface deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void RL_Sim2RealHardwareInterface::state_callback(
    const rl_sim2real_msgs::msg::RobotState::SharedPtr msg)
{
  last_received_sequence_ = msg->sequence;
  has_feedback_ = true;
  last_feedback_time_ = std::chrono::steady_clock::now();

  // Store velocity (fixed array of 3)
  for (std::size_t i = 0; i < kRobotVelocityCount; ++i)
  {
    robot_velocity_[i] = msg->velocity[i];
  }

  // Store IMU data: orientation (3), angular_velocity (3), linear_acceleration
  // (3)
  std::size_t imu_index = 0;
  for (std::size_t i = 0; i < 3; ++i)
  {
    imu_values_[imu_index++] = msg->orientation[i];
  }
  for (std::size_t i = 0; i < 3; ++i)
  {
    imu_values_[imu_index++] = msg->angular_velocity[i];
  }
  for (std::size_t i = 0; i < 3; ++i)
  {
    imu_values_[imu_index++] = msg->linear_acceleration[i];
  }

  mcu_timestamp_ms_ = msg->timestamp_ms;
  error_state_ = msg->error_state;

  // Debug logging
  // auto now = std::chrono::steady_clock::now();
  // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
  //     now - last_debug_log_time_);
  // if (elapsed.count() >= 1000)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger(node_name_),
  //               "[RX] seq=%u vel=[%.2f %.2f %.2f] orient(rad)=[%.3f %.3f
  //               %.3f] " "gyro(rad/s)=[%.3f %.3f %.3f] accel(m/s2)=[%.2f %.2f
  //               %.2f] " "err=0x%02X", msg->sequence, robot_velocity_[0],
  //               robot_velocity_[1], robot_velocity_[2], imu_values_[0],
  //               imu_values_[1], imu_values_[2], imu_values_[3],
  //               imu_values_[4], imu_values_[5], imu_values_[6],
  //               imu_values_[7], imu_values_[8], error_state_);
  //   last_debug_log_time_ = now;
  // }
}

hardware_interface::return_type RL_Sim2RealHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RL_Sim2RealHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (!command_publisher_)
  {
    RCLCPP_ERROR(rclcpp::get_logger(node_name_),
                 "Command publisher not initialized");
    return hardware_interface::return_type::ERROR;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now < next_command_publish_time_)
  {
    return hardware_interface::return_type::OK;
  }

  // Prepare joint command message
  auto msg = rl_sim2real_msgs::msg::JointCommand();
  msg.sequence = sequence_++;

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    msg.positions[i] = static_cast<float>(joint_commands_[i]);
  }

  // Publish command
  command_publisher_->publish(msg);
  last_sent_sequence_ = msg.sequence;
  do
  {
    next_command_publish_time_ += kCommandPublishPeriod;
  } while (next_command_publish_time_ <= now);

  // Debug logging
  // auto now = std::chrono::steady_clock::now();
  // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
  //     now - last_debug_log_time_);
  // if (elapsed.count() >= 1000)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger(node_name_),
  //               "[TX] seq=%u joints(rad)=[%.3f %.3f %.3f %.3f %.3f %.3f "
  //               "%.3f %.3f %.3f %.3f %.3f %.3f]",
  //               msg.sequence, msg.positions[0], msg.positions[1],
  //               msg.positions[2], msg.positions[3], msg.positions[4],
  //               msg.positions[5], msg.positions[6], msg.positions[7],
  //               msg.positions[8], msg.positions[9], msg.positions[10],
  //               msg.positions[11]);
  // }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
RL_Sim2RealHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export IMU interfaces (9 values)
  state_interfaces.emplace_back("imu", "orientation.x", &imu_values_[0]);
  state_interfaces.emplace_back("imu", "orientation.y", &imu_values_[1]);
  state_interfaces.emplace_back("imu", "orientation.z", &imu_values_[2]);
  state_interfaces.emplace_back("imu", "angular_velocity.x", &imu_values_[3]);
  state_interfaces.emplace_back("imu", "angular_velocity.y", &imu_values_[4]);
  state_interfaces.emplace_back("imu", "angular_velocity.z", &imu_values_[5]);
  state_interfaces.emplace_back("imu", "linear_acceleration.x",
                                &imu_values_[6]);
  state_interfaces.emplace_back("imu", "linear_acceleration.y",
                                &imu_values_[7]);
  state_interfaces.emplace_back("imu", "linear_acceleration.z",
                                &imu_values_[8]);
  state_interfaces.emplace_back("remote_control", "linear.x",
                                &robot_velocity_[0]);
  state_interfaces.emplace_back("remote_control", "linear.y",
                                &robot_velocity_[1]);
  state_interfaces.emplace_back("remote_control", "angular.z",
                                &robot_velocity_[2]);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RL_Sim2RealHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export joint command interfaces (12 joints)
  const std::vector<std::string> joint_names = {
      "front_left_hip_joint",    "front_left_thigh_joint",
      "front_left_calf_joint",   "front_right_hip_joint",
      "front_right_thigh_joint", "front_right_calf_joint",
      "back_left_hip_joint",     "back_left_thigh_joint",
      "back_left_calf_joint",    "back_right_hip_joint",
      "back_right_thigh_joint",  "back_right_calf_joint"};

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    command_interfaces.emplace_back(joint_names[i],
                                    hardware_interface::HW_IF_POSITION,
                                    &joint_commands_[i]);
  }

  return command_interfaces;
}

}  // namespace rl_sim2real

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rl_sim2real::RL_Sim2RealHardwareInterface,
                       hardware_interface::SystemInterface)
