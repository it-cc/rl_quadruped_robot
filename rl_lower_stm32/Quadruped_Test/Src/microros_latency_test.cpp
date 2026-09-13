#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include "api_serial.h"
#include "microros_latency_test.h"

extern "C"
{
  bool cubemx_transport_open(struct uxrCustomTransport* transport);
  bool cubemx_transport_close(struct uxrCustomTransport* transport);
  size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                                const uint8_t* buf, size_t len, uint8_t* err);
  size_t cubemx_transport_read(struct uxrCustomTransport* transport,
                               uint8_t* buf, size_t len, int timeout,
                               uint8_t* err);
}

namespace test
{
namespace
{
MicroRosLatencyTest* g_latency_test = nullptr;

void ping_callback_wrapper(const void* msgin)
{
  if (g_latency_test != nullptr) g_latency_test->pingCallback(msgin);
}
}  // namespace

MicroRosLatencyTest::MicroRosLatencyTest(UART_HandleTypeDef& micro_ros_uart)
    : task::ManagedTask("micro_ros_latency", 20U, 3000U,
                        task::TaskType::TASK_PERIOD, 1U),
      micro_ros_uart_(micro_ros_uart)
{
}

void MicroRosLatencyTest::init()
{
  g_latency_test = this;
  initialized_ = true;
}

bool MicroRosLatencyTest::initMessages()
{
  if (!std_msgs__msg__UInt64MultiArray__init(&ping_message_) ||
      !std_msgs__msg__UInt64MultiArray__init(&pong_message_))
  {
    return false;
  }

  ping_message_.data.data = ping_data_;
  ping_message_.data.size = 0U;
  ping_message_.data.capacity = kPingFieldCount;
  pong_message_.data.data = pong_data_;
  pong_message_.data.size = kPongFieldCount;
  pong_message_.data.capacity = kPongFieldCount;
  return true;
}

bool MicroRosLatencyTest::synchronizeTime()
{
  constexpr int kSyncAttempts = 3;
  constexpr int kSyncTimeoutMs = 1000;
  for (int attempt = 0; attempt < kSyncAttempts; ++attempt)
  {
    if (rmw_uros_sync_session(kSyncTimeoutMs) == RMW_RET_OK &&
        rmw_uros_epoch_synchronized())
    {
      return true;
    }
    osDelay(100);
  }
  return false;
}

bool MicroRosLatencyTest::initMicroRos()
{
  if (rmw_uros_set_custom_transport(
          true, &micro_ros_uart_, cubemx_transport_open, cubemx_transport_close,
          cubemx_transport_write, cubemx_transport_read) != RMW_RET_OK)
  {
    uart_printf("[latency-test] transport setup failed\r\n");
    return false;
  }

  uart_printf("[latency-test] waiting for micro-ROS agent\r\n");
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) osDelay(200);

  static rcl_allocator_t allocator = rcl_get_default_allocator();
  static rclc_support_t support;
  static rcl_node_t node;
  static rcl_publisher_t pong_publisher;
  static rcl_subscription_t ping_subscriber;
  static rclc_executor_t executor;

  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK ||
      rclc_node_init_default(&node, "microros_latency_test", "", &support) !=
          RCL_RET_OK ||
      rclc_publisher_init_default(
          &pong_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt64MultiArray),
          "latency_test/pong") != RCL_RET_OK ||
      rclc_subscription_init_default(
          &ping_subscriber, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt64MultiArray),
          "latency_test/ping") != RCL_RET_OK ||
      !initMessages() ||
      rclc_executor_init(&executor, &support.context, 1, &allocator) !=
          RCL_RET_OK ||
      rclc_executor_add_subscription(&executor, &ping_subscriber,
                                     &ping_message_, ping_callback_wrapper,
                                     ON_NEW_DATA) != RCL_RET_OK)
  {
    uart_printf("[latency-test] micro-ROS entity setup failed\r\n");
    return false;
  }

  state_publisher_ = &pong_publisher;
  executor_ = &executor;
  time_synchronized_ = synchronizeTime();
  uart_printf("[latency-test] time sync %s\r\n",
              time_synchronized_ ? "succeeded" : "failed");
  return true;
}

void MicroRosLatencyTest::pingCallback(const void* msgin)
{
  const auto* ping = static_cast<const std_msgs__msg__UInt64MultiArray*>(msgin);
  if (ping == nullptr || ping->data.size < kPingFieldCount ||
      state_publisher_ == nullptr)
  {
    return;
  }

  const uint64_t mcu_receive_time_ns =
      static_cast<uint64_t>(rmw_uros_epoch_nanos());
  const bool synchronized = time_synchronized_ && rmw_uros_epoch_synchronized();
  pong_message_.data.data[0] = ping->data.data[0];
  pong_message_.data.data[1] = ping->data.data[1];
  pong_message_.data.data[2] = mcu_receive_time_ns;
  pong_message_.data.data[4] = synchronized ? 1U : 0U;
  pong_message_.data.data[3] = static_cast<uint64_t>(rmw_uros_epoch_nanos());

  const rcl_ret_t publish_result = rcl_publish(
      static_cast<rcl_publisher_t*>(state_publisher_), &pong_message_, nullptr);
  (void)publish_result;
}

void MicroRosLatencyTest::taskProcess()
{
  if (!initialized_) return;

  if (!microros_initialized_)
  {
    microros_initialized_ = initMicroRos();
    if (!microros_initialized_) osDelay(1000);
    return;
  }

  if (executor_ != nullptr)
  {
    (void)rclc_executor_spin_some(static_cast<rclc_executor_t*>(executor_),
                                  RCL_MS_TO_NS(1));
  }
}
}  // namespace test
