#ifndef MICROROS_LATENCY_TEST_H
#define MICROROS_LATENCY_TEST_H

#ifdef __cplusplus

#include <cstdint>

#include "api_task.h"
#include "std_msgs/msg/u_int64_multi_array.h"

namespace test
{
class MicroRosLatencyTest : public task::ManagedTask
{
 public:
  explicit MicroRosLatencyTest(UART_HandleTypeDef& micro_ros_uart);

  void init();
  void pingCallback(const void* msgin);

 protected:
  void taskProcess() override;

 private:
  static constexpr std::size_t kPingFieldCount = 2;
  static constexpr std::size_t kPongFieldCount = 5;

  bool initMicroRos();
  bool initMessages();
  bool synchronizeTime();

  UART_HandleTypeDef& micro_ros_uart_;
  void* state_publisher_{nullptr};
  void* executor_{nullptr};
  bool initialized_{false};
  bool microros_initialized_{false};
  bool time_synchronized_{false};
  uint64_t ping_data_[kPingFieldCount]{};
  uint64_t pong_data_[kPongFieldCount]{};
  std_msgs__msg__UInt64MultiArray ping_message_{};
  std_msgs__msg__UInt64MultiArray pong_message_{};
};
}  // namespace test

#endif  // __cplusplus

#endif  // MICROROS_LATENCY_TEST_H
