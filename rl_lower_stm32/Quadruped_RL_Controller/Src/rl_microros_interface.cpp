#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rl_sim2real_msgs/msg/joint_command.h>
#include <rl_sim2real_msgs/msg/robot_state.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include <cmath>
#include <cstring>

#include "main.h"
#include "rl_microros_interface.h"

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

namespace rl_controller
{
namespace
{
constexpr float kDegreesToRadians = 0.01745329252F;
constexpr float kRadiansToDegrees = 57.2957795131F;
constexpr float kStandardGravity = 9.80665F;
constexpr float kRemoteLinearXScale = 0.3F;
constexpr float kRemoteLinearYScale = 0.3F;
constexpr float kRemoteAngularZScale = 1.0F;

constexpr std::array<float, 12> kDefaultJointPosition = {
    0.0, 0.6151, -0.9065, 0.0, 0.6151, -0.9065,
    0.0, 0.6519, -0.9709, 0.0, 0.6519, -0.9709};

RL_MicroRosInterface* g_interface_instance = nullptr;

void command_callback_wrapper(const void* msgin)
{
  if (g_interface_instance != nullptr)
  {
    g_interface_instance->commandCallback(msgin);
  }
}
}  // namespace

RL_MicroRosInterface::RL_MicroRosInterface(
    UART_HandleTypeDef& micro_ros_uart, UART_HandleTypeDef& imu_uart,
    UART_HandleTypeDef& remote_control_uart)
    : task::ManagedTask("RL_MicroROS", 5U, 3000U, task::TaskType::TASK_PERIOD,
                        4U),
      micro_ros_uart_(micro_ros_uart),
      imu_(imu_uart),
      remote_control_(remote_control_uart),
      servo_()
{
  commanded_positions_.fill(0.0F);
}

void RL_MicroRosInterface::init()
{
  imu_.start();
  remote_control_.start();
  servo_.init();
  servo_.setAllServoAngles(kDefaultJointPosition.data());
  initialized_ = true;
  g_interface_instance = this;
}

void RL_MicroRosInterface::initMicroRos()
{
  uart_printf("[micro-ROS] Initialization start\r\n");
  rmw_ret_t rmw_ret = rmw_uros_set_custom_transport(
      true, &micro_ros_uart_, cubemx_transport_open, cubemx_transport_close,
      cubemx_transport_write, cubemx_transport_read);

  uart_printf("[micro-ROS] start ping agent\r\n");
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
  {
    osDelay(200);
  }
  uart_printf("[micro-ROS] connect to agent\r\n");
  static rclc_support_t support;
  static rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  support_ = &support;
  static rcl_node_t node;
  rclc_node_init_default(&node, "rl_microros_node", "", &support);
  node_ = &node;

  // Create publisher for robot state
  static rcl_publisher_t state_publisher;
  rclc_publisher_init_default(
      &state_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(rl_sim2real_msgs, msg, RobotState),
      "robot_state");
  state_publisher_ = &state_publisher;

  // Create subscription for joint command
  static rcl_subscription_t command_subscriber;
  rclc_subscription_init_default(
      &command_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(rl_sim2real_msgs, msg, JointCommand),
      "joint_command");
  command_subscriber_ = &command_subscriber;

  // Create executor
  static rclc_executor_t executor;
  static rl_sim2real_msgs__msg__JointCommand command_msg;
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &command_subscriber, &command_msg,
                                 command_callback_wrapper, ON_NEW_DATA);

  executor_ = &executor;
  microros_initialized_ = true;

  uart_printf("[micro-ROS] Initialization complete\r\n");
}

void RL_MicroRosInterface::commandCallback(const void* msgin)
{
  const auto* msg =
      static_cast<const rl_sim2real_msgs__msg__JointCommand*>(msgin);

  // Store commanded positions
  for (size_t i = 0; i < 12; ++i)
  {
    commanded_positions_[i] = msg->positions[i];
  }

  servo_.setAllServoAngles(commanded_positions_.data());
}

void RL_MicroRosInterface::publishState()
{
  if (!microros_initialized_ || state_publisher_ == nullptr) return;

  rl_sim2real_msgs__msg__RobotState state_msg;
  state_msg.sequence = last_sequence_++;
  // Velocity from remote control
  state_msg.velocity[0] =
      remote_control_.isOnline()
          ? (remote_control_.getChannel(2) > 1550   ? kRemoteLinearXScale
             : remote_control_.getChannel(2) < 1450 ? -kRemoteLinearXScale
                                                    : 0.0F)
          : 0.0F;
  state_msg.velocity[1] =
      remote_control_.isOnline()
          ? (remote_control_.getChannel(3) > 1550   ? kRemoteLinearYScale
             : remote_control_.getChannel(3) < 1450 ? -kRemoteLinearYScale
                                                    : 0.0F)
          : 0.0F;
  state_msg.velocity[2] =
      remote_control_.isOnline()
          ? (remote_control_.getChannel(0) > 1550   ? kRemoteAngularZScale
             : remote_control_.getChannel(0) < 1450 ? -kRemoteAngularZScale
                                                    : 0.0F)
          : 0.0F;

  // IMU data
  const types::Euler angle = imu_.getAngle();
  const types::Gyroscope gyro = imu_.getAngularVelocity();
  const types::Accelerometer accel = imu_.getAcceleration();

  // orientation[0] is roll,orientation[1] is pitch,adapt to imu mounting orientation
  state_msg.orientation[0] = angle.pitch * kDegreesToRadians;
  state_msg.orientation[1] = angle.roll * kDegreesToRadians;
  state_msg.orientation[2] = -angle.yaw * kDegreesToRadians;

  state_msg.angular_velocity[0] = gyro.y * kDegreesToRadians;
  state_msg.angular_velocity[1] = gyro.x * kDegreesToRadians;
  state_msg.angular_velocity[2] = -gyro.z * kDegreesToRadians;

  state_msg.linear_acceleration[0] = accel.y * kStandardGravity;
  state_msg.linear_acceleration[1] = accel.x * kStandardGravity;
  state_msg.linear_acceleration[2] = -accel.z * kStandardGravity;

  state_msg.timestamp_ms = HAL_GetTick();
  state_msg.error_state = error_state_;

  // Publish
  rcl_ret_t ret = rcl_publish(static_cast<rcl_publisher_t*>(state_publisher_),
                              &state_msg, NULL);

  // Debug output
  // static uint32_t last_print_ms = 0;
  // const uint32_t now = HAL_GetTick();
  // if (static_cast<uint32_t>(now - last_print_ms) >= 2000U)
  // {
  //   uart_printf(
  //       "[micro-ROS TX] seq=%u vel=[%.2f %.2f %.2f] orient=[%.3f %.3f "
  //       "%.3f] ang_vel=[%.3f %.3f %.3f] accel(m/s2)=[%.2f %.2f %.2f]\r\n",
  //       state_msg.sequence, static_cast<double>(state_msg.velocity[0]),
  //       state_msg.velocity[1], state_msg.velocity[2],
  //       state_msg.orientation[0], state_msg.orientation[1],
  //       state_msg.orientation[2], state_msg.angular_velocity[0],
  //       state_msg.angular_velocity[1], state_msg.angular_velocity[2],
  //       state_msg.linear_acceleration[0], state_msg.linear_acceleration[1],
  //       state_msg.linear_acceleration[2]);
  //   last_print_ms = now;
  // }
}

void RL_MicroRosInterface::taskProcess()
{
  if (!initialized_) return;

  // Initialize micro-ROS on first run (after RTOS scheduler starts)
  if (!microros_initialized_)
  {
    initMicroRos();
    if (!microros_initialized_)
    {
      // Failed to initialize, retry next time
      osDelay(1000);
      return;
    }
  }
  // Spin executor to process incoming messages
  if (microros_initialized_ && executor_ != nullptr)
  {
    rclc_executor_spin_some(static_cast<rclc_executor_t*>(executor_),
                            RCL_MS_TO_NS(1));
  }

  const uint32_t now = HAL_GetTick();
  if (next_state_publish_ms_ == 0U)
    next_state_publish_ms_ = now;

  if (static_cast<int32_t>(now - next_state_publish_ms_) >= 0)
  {
    do
    {
      next_state_publish_ms_ += kStatePublishPeriodMs;
    } while (static_cast<int32_t>(now - next_state_publish_ms_) >= 0);
    publishState();
  }
}

}  // namespace rl_controller
