#ifndef RL_MICROROS_INTERFACE_H
#define RL_MICROROS_INTERFACE_H

#ifdef __cplusplus

#include <array>
#include <cstdint>

#include "FlySkyIbus.h"
#include "api_task.h"
#include "imu_JY901S.h"
#include "rl_servo.h"

namespace rl_controller
{
class RL_MicroRosInterface : public task::ManagedTask
{
 public:
  RL_MicroRosInterface(UART_HandleTypeDef& micro_ros_uart,
                       UART_HandleTypeDef& imu_uart,
                       UART_HandleTypeDef& remote_control_uart);

  void init();
  void commandCallback(const void* msgin);

 protected:
  void taskProcess() override;

 private:
  void initMicroRos();
  void publishState();

  imu::JY901S imu_;
  remote_control::FlySkyIbus remote_control_;
  RL_Servo servo_;

  UART_HandleTypeDef& micro_ros_uart_;

  void* support_{nullptr};
  void* node_{nullptr};
  void* state_publisher_{nullptr};
  void* command_subscriber_{nullptr};
  void* executor_{nullptr};

  uint32_t error_state_{0U};
  uint8_t last_sequence_{0U};
  uint32_t last_command_ms_{0U};
  uint32_t last_debug_print_ms_{0U};
  bool microros_initialized_{false};
  bool initialized_{false};

  std::array<float, 12> commanded_positions_{};
};
}  // namespace rl_controller

#endif  // __cplusplus
#endif  // RL_MICROROS_INTERFACE_H
